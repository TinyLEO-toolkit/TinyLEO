"""
SRv6 Routing Management Script
This script manages SRv6 routing for ground stations (GS) and satellites (SH).
It includes functionalities for:
- Route updates and segment list initialization.
- File watching for configuration changes.
- Packet processing using NetfilterQueue.
- Managing routing tables and SRv6 encapsulation.
"""

import threading
from netfilterqueue import NetfilterQueue
from pyroute2 import IPRoute
from scapy.layers.inet6 import IPv6,IPv6ExtHdrSegmentRouting,ICMPv6TimeExceeded
import ipaddress
import select
from utils import *

cell_width = 11

class GsRouteManager:
    """
    A class to manage SRv6 routing for ground stations (GS).
    
    Attributes:
        hostname (str): The hostname of the ground station.
        iproute (IPRoute): An instance of pyroute2.IPRoute for managing routes.
        route_update_event (RouteUpdateEvent): Event to trigger route updates.
        route_tables (dict): Mapping of table IDs to table names.
        table_id (int): Current routing table ID in use.
        table_name (str): Name of the current routing table.
    """

    def __init__(self, hostname):
        """
        Initializes the GsRouteManager instance.

        Args:
            hostname (str): The hostname of the ground station.
        """
        self.hostname = hostname
        self.iproute = IPRoute()
        subprocess.run(["sudo", "tee", "/etc/hostname"], input=f"{hostname}\n", text=True)
        subprocess.run(["sudo", "sed", "-i", f"s/ubuntu/{hostname}/g", "/etc/hosts"])
        self.route_update_event = RouteUpdateEvent()
        self.route_tables = {200: "gs_dy_route1", 201: "gs_dy_route2"}
        for table_id, table_name in self.route_tables.items():
            create_route_table(self.iproute, table_id, table_name)
        self.table_id = 200
        self.table_name = self.route_tables[self.table_id]
        add_route_rule(self.iproute,201)
        
        self.start_file_watcher()
        while True:
            self.reload_routes()
         
    def route_update(self):
        """
        Updates the routing table by switching between two tables (200 and 201),
        flushing the current table, and reinitializing routes.
        """
        start_time = time.time()
        self.ts,self.sat,self.sat_ip,self.sat_mac,self.seg_list = init_gs_state_from_memory(self.hostname)
        # self.ts,self.sat,self.sat_ip,self.sat_mac,self.seg_list = init_gs_state_from_file()
        pre_table_id = self.table_id
        self.table_id = 201 if pre_table_id == 200 else 200
        
        flush_table(self.table_id)
        self.init_gsl()
        self.init_seg_list()
        self.init_default_sat()
        if self.table_id == 200:
            add_route_rule(self.iproute,200,priority=99)
        else:
            delete_route_rule(self.iproute,200,priority=99)
        print(f"Route update time: {time.time()-start_time}")
    
    def init_default_sat(self):
        """
        Initializes the default satellite route in the routing table.
        """
        idx = self.iproute.link_lookup(ifname=self.sat)[0]
        self.iproute.route("add",dst="ce::/16",table=self.table_id, gateway=self.sat_ip, oif=idx, priority=1024)

    def ts_changed_trigger(self):
        """
        Triggers a route update event when the timestamp changes.
        """
        self.route_update_event.set()

    def flush_route_trigger(self):
        """
        Placeholder for triggering a route flush event.
        """
        pass

    def reload_routes(self):
        """
        Reloads routes by clearing the route update event, updating routes, and listening for changes.
        """
        self.route_update_event.clear()
        self.route_update() 
        self.listen()

    def listen(self):
        """
        Listens for route update events using a file descriptor.
        """
        route_update_fd = self.route_update_event.get_fd()
        read_fds, _, _ = select.select([route_update_fd], [], [])#

    def start_file_watcher(self):
        """
        Starts a file watcher to monitor changes in the controller directory and trigger route updates.
        """
        event_handler = RouteUpdateHandler(self)
        observer = Observer()
        observer.schedule(event_handler, path="/resources/controller/", recursive=False)
        observer.daemon = True
        observer.start()

    def init_gsl(self):
        """
        Initializes the ground station link (GSL) by adding routes and neighbors for the satellite interface.
        """
        try:
            gs_idx = self.iproute.link_lookup(ifname=self.sat)[0]
            self.iproute.route("add",table=self.table_id,dst=self.sat_ip,oif=gs_idx, priority=1)
            self.iproute.neigh('add',dst=self.sat_ip,lladdr=self.sat_mac,ifindex=gs_idx,state=0x80)
        except Exception as e:
            pass
        
    def init_seg_list(self):
        """
        Initializes the segment list for SRv6 routing by adding routes with SRv6 encapsulation.
        """
        this_cell = ip2cell(self.sat_ip)
        for key in self.seg_list:
            src,dst = seg_key_reverse(key)
            if src==this_cell:
                segs = []
                for cell in reversed(self.seg_list[key]):
                    segs.append(f"ce:{cell[0]}:ce:{cell[1]}::cccc")
                segs = ",".join(segs)
                idx = self.iproute.link_lookup(ifname=self.sat)[0]
                self.iproute.route("add",
                    table=self.table_id,
                    dst=f"ce:{dst[0]}:ce:{dst[1]}::/64",  
                    encap={"type": "seg6", "mode": "inline", "segs": segs},  # SRv6 Encapsulation
                    gateway=self.sat_ip,    # next hop
                    oif=idx,
                    priority = 1)  
            elif dst==this_cell:
                segs = []
                for cell in self.seg_list[key]:
                    segs.append(f"ce:{cell[0]}:ce:{cell[1]}::cccc")
                segs = ",".join(segs)
                idx = self.iproute.link_lookup(ifname=self.sat)[0]
                self.iproute.route("add",
                    table=self.table_id,
                    dst=f"ce:{src[0]}:ce:{src[1]}::/64",
                    encap={"type": "seg6", "mode": "inline", "segs": segs},  # SRv6 Encapsulation
                    gateway=self.sat_ip,    
                    oif=idx,
                    priority = 1)      

class SatRouteManager:
    """
    A class to manage SRv6 routing for satellites.

    Attributes:
        hostname (str): The hostname of the satellite.
        iproute (IPRoute): An instance of pyroute2.IPRoute for managing routes.
        nfqueues (list): A list of NetfilterQueue instances for packet processing.
        route_update_event (RouteUpdateEvent): Event to trigger route updates.
        table_id (int): The ID of the routing table used for satellite routes.
        table_name (str): The name of the routing table.
        route_table (set): A set to store active routes.
        link_damage_signal (bool): A flag indicating if a link damage event occurred.
        flush_route_signal (bool): A flag indicating if a route flush event occurred.
        bent_pipe (bool): A flag indicating if the satellite is in bent-pipe mode.
    """

    def __init__(self,hostname):
        """
        Initializes the SatRouteManager instance.

        Args:
            hostname (str): The hostname of the satellite.
        """
        self.hostname = hostname
        self.iproute = IPRoute()
        self.nfqueues = [NetfilterQueue() for _ in range(2)] 
        subprocess.run(["sudo", "tee", "/etc/hostname"], input=f"{hostname}\n", text=True)
        subprocess.run(["sudo", "sed", "-i", f"s/ubuntu/{hostname}/g", "/etc/hosts"])
        self.route_update_event = RouteUpdateEvent()
        # self.all_isls,self.all_gsls,self.all_inter_cell_isls,self.all_sat_cell,self.all_cell_rings = init_all_sat_state()
        self.table_id = 200
        self.table_name = "sat_dy_route"
        self.route_table = set()
        # self.packet_count = 0
        # self.ts = get_ts()-1
        self.link_damage_signal = False
        self.flush_route_signal = False
        self.bent_pipe = False
        clear_iptables()
        self.init_iptables()
        self.init_cell_nic()
        create_route_table(self.iproute, self.table_id, self.table_name)
        self.route_update()
        self.bind_nfqueue()
        self.start_file_watcher()
        self.start()
        
        while True:
            self.reload_routes()

    def init_cell_nic(self):
        """
        Initializes a dummy network interface for the satellite's cell.
        """
        iface_name = "CELL"
        try:
            self.iproute.link("add", ifname=iface_name, kind="dummy")
            self.iproute.link("set", ifname=iface_name, state="up")
        except Exception as e:
            pass

    def route_update(self):
        """
        Updates the satellite's routing table by clearing the current table,
        reinitializing routes, and adding new rules.
        """
        # start_time = time.time()
        self.update_sat_state()
        if self.bent_pipe:
            time.sleep(0.5)
        delete_route_rule(self.iproute,self.table_id)
        flush_table(self.table_id)
        self.route_table.clear()
        self.init_route_table()
        add_route_rule(self.iproute, self.table_id)
        # print(f"Route update time: {time.time()-start_time}")

    def update_intra_cell_sats(self):
        """
        Updates the next and previous satellites in the same cell ring.
        """
        try:
            index = self.cell_ring.index(self.hostname)
        except ValueError:
            self.next_sat,self.pre_sat = None,None
            return
        self.next_sat = self.cell_ring[(index - 1) % len(self.cell_ring)]
        if self.next_sat == self.hostname:
            self.next_sat = None
        self.pre_sat = self.cell_ring[(index + 1) % len(self.cell_ring)]
        if self.pre_sat == self.hostname:
            self.pre_sat = None

    def init_route_table(self):
        """
        Initializes the routing table with routes for ground stations (GSL),
        inter-cell links, and intra-cell links.
        """
        if self.bent_pipe:
            return
        self.update_cell_sid()
        # Add routes for ground stations
        for gs in self.gsls:
            gs_ip,gs_mac = self.gsls[gs][0],self.gsls[gs][1]
            try:
                gs_idx = self.iproute.link_lookup(ifname=gs)[0]
                self.iproute.route("add",table=self.table_id,dst=gs_ip,oif=gs_idx, priority=1)
                self.iproute.neigh('add',dst=gs_ip,lladdr=gs_mac,ifindex=gs_idx,state=0x80)
            except Exception as e:
                pass


        # Add routes for inter-cell links
        if self.inter_cell_isls!={}:
            inter_cell_sat = self.inter_cell_isls['near_sat']
            if inter_cell_sat in self.isls:
                near_cell = self.inter_cell_isls['near_cell']
                near_cell_network = str(ipaddress.IPv6Network(f"ce:{near_cell[0]}:ce:{near_cell[1]}::cccc/64", strict=False))
                inter_cell_sat_ip,inter_cell_sat_mac  =  self.isls[inter_cell_sat][0],self.isls[inter_cell_sat][1]
                try:
                    inter_cell_idx = self.iproute.link_lookup(ifname=inter_cell_sat)[0]
                    self.iproute.route("add",table=self.table_id,dst=near_cell_network,gateway=inter_cell_sat_ip, oif=inter_cell_idx, priority=1)
                    self.iproute.neigh('add',dst=inter_cell_sat_ip,lladdr=inter_cell_sat_mac,ifindex=inter_cell_idx,state=0x80)
                except Exception as e:
                    pass

        # Add routes for intra-cell links
        if self.next_sat is not None:
            default_sat_ip,default_sat_mac = self.isls[self.next_sat][0],self.isls[self.next_sat][1]
            try:
                default_idx = self.iproute.link_lookup(ifname=self.next_sat)[0]
                self.iproute.route("add",dst="ce::/16",table=self.table_id, gateway=default_sat_ip, oif=default_idx, priority=1024)
                self.iproute.neigh('add',dst=default_sat_ip,lladdr=default_sat_mac,ifindex=default_idx,state=0x80)
            except Exception as e:
                pass

    def update_sat_state(self):
        """
        Updates the satellite's state, including ISLs, GSLs, inter-cell ISLs,
        and cell ring information.
        """
        self.bent_pipe = False
        if self.link_damage_signal:
            self.link_damage_signal = False
            self.ts,self.isls,self.gsls,self.inter_cell_isls,self.sat_cell,self.cell_ring = init_sat_state_from_memory(self.hostname)
            # self.ts,self.isls,self.gsls,self.inter_cell_isls,self.sat_cell,self.cell_ring = init_sat_state_from_file()
        else:
            if self.flush_route_signal:
                self.flush_route_signal = False
            else:
                self.ts,self.isls, self.gsls, self.inter_cell_isls, self.sat_cell,self.cell_ring = init_sat_state_from_memory(self.hostname)
                # self.ts,self.isls, self.gsls, self.inter_cell_isls, self.sat_cell,self.cell_ring = init_sat_state_from_file()
        if self.isls == {}:
            self.bent_pipe = True
        self.update_intra_cell_sats()
        
    def bind_nfqueue(self):
        """
        Binds NetfilterQueue instances to handle packet processing.
        """
        self.nfqueues[0].bind(1, self.preroute_dst_unreachable)
        self.nfqueues[1].bind(2, self.output_time_exceeded)

    def listen(self):
        """
        Listens for route update events using a file descriptor.
        """
        route_update_fd = self.route_update_event.get_fd()
        read_fds, _, _ = select.select([route_update_fd], [], [])#

    def start(self):
        """
        Starts threads for handling NetfilterQueue packet processing.
        """
        self.threads = []
        for nfqueue_id,nfqueue in enumerate(self.nfqueues):
            thread = threading.Thread(target=self.run_nfqueue, args=(nfqueue_id,))
            thread.start()
            self.threads.append(thread)
        for thread in self.threads:
            thread.join()

    def run_nfqueue(self, nfqueue_id):
        """
        Runs a specific NetfilterQueue instance for packet processing.

        Args:
            nfqueue_id (int): The ID of the NetfilterQueue instance.
        """
        nfqueue_fd = self.nfqueues[nfqueue_id].get_fd()
        route_update_fd = self.route_update_event.get_fd()
        while True:
            try:
                read_fds, _, _ = select.select([nfqueue_fd,route_update_fd], [], [])#
                if route_update_fd in read_fds:
                    break
                if nfqueue_fd in read_fds:
                    self.nfqueues[nfqueue_id].run(block=False)
            except Exception as e:
                pass

    def unbind_nfqueues(self):
        """
        Unbinds all NetfilterQueue instances and stops their threads.
        """
        for thread in self.threads:
            thread.join()
        for nfqueue in self.nfqueues:
            nfqueue.unbind()
            
    def ts_changed_trigger(self):
        """
        Triggers a route update event when the timestamp changes.
        """
        self.route_update_event.set()

    def link_changed_trigger(self):
        """
        Triggers a route update event when a link damage event occurs.
        """
        self.link_damage_signal = True
        self.route_update_event.set()
    
    def flush_route_trigger(self):
        """
        Triggers a route flush event.
        """
        self.flush_route_signal = True
        self.route_update_event.set()

    def reload_routes(self):
        """
        Reloads routes and restarts NetfilterQueue bindings if necessary.
        """
        self.route_update()
        self.route_update_event.clear()
        if self.bent_pipe:
            self.listen()
        else:
            self.start()
    
    def start_file_watcher(self):
        """
        Starts file watchers to monitor changes in the controller directory
        and trigger route updates or link damage events.
        """
        route_update_handler = RouteUpdateHandler(self)
        observer1 = Observer()
        observer1.schedule(route_update_handler, path="/resources/controller/", recursive=False)
        observer1.daemon = True
        observer1.start()
        link_damage_handler = LinkDamageHandler(self)
        observer2 = Observer()
        observer2.schedule(link_damage_handler, path="/resources/", recursive=False)
        observer2.daemon = True
        observer2.start()

    
    def update_cell_sid(self):
        """
        Updates the satellite's cell SID (Segment Identifier) in the routing table.
        """
        sat_cell = self.sat_cell
        if not sat_cell:
            return
        try:
            iface_index = self.iproute.link_lookup(ifname='CELL')[0]
            self.iproute.route("add",table=self.table_id,dst=f"ce:{sat_cell[0]}:ce:{sat_cell[1]}::cccc",encap={"type": "seg6local", "action": "End"},oif=iface_index, priority=1)
        except Exception as e:
            pass

    
    def init_iptables(self):
        """
        Initializes iptables rules for handling ICMPv6 and unreachable packets.
        """
        add_ip6tables_rule('OUTPUT', dst="ce::/16", protocol="icmpv6", icmpv6_type="time-exceeded", queue_num='2')
        add_ip6tables_rule('PREROUTING', table_name="mangle", dst="ce::/16", addrtype=True, dst_type="UNREACHABLE", queue_num='1')
    
    def preroute_dst_unreachable(self,packet):
        """
        Handles packets with unreachable destinations by adding SRv6 segments.

        Args:
            packet: The packet to process.
        """
        pkt = IPv6(packet.get_payload())
        dst_cell = ip2cell(pkt.dst)
        dis = get_cell_dis(self.sat_cell,dst_cell)
        if dis <= 1:
            pkt.hlim = 255
            packet.set_payload(bytes(pkt))
            packet.accept()
            return
        redirect_cell_route = get_cell_route(self.sat_cell,dst_cell)
        # print("redirect_cell_route:",redirect_cell_route)
        # deal with seg6
        if pkt.nh==43 and IPv6ExtHdrSegmentRouting in pkt:
            segleft = pkt[IPv6ExtHdrSegmentRouting].segleft
            address_list = pkt[IPv6ExtHdrSegmentRouting].addresses
            payload = pkt[IPv6ExtHdrSegmentRouting].payload
            new_address_list = address_list[:segleft+1]
            for item in reversed(redirect_cell_route):
                new_address_list.append(f"ce:{item[0]}:ce:{item[1]}::cccc")
                segleft += 1
            new_address_list.append(f"a::a")
            new_srh = IPv6ExtHdrSegmentRouting(
                addresses=new_address_list,
                segleft=segleft,
                nh=pkt[IPv6ExtHdrSegmentRouting].nh
            )
            new_pkt = IPv6(src=pkt.src, dst=new_address_list[segleft])/new_srh/ payload
            packet.set_payload(bytes(new_pkt))
        packet.accept()

    def output_time_exceeded(self,packet):
        """
        Handles ICMPv6 Time Exceeded packets by adding SRv6 segments.

        Args:
            packet: The packet to process.
        """
        out_iface = packet.outdev
        pkt = IPv6(packet.get_payload()) 
        dst = pkt.dst
        dst_cell = ip2cell(pkt.dst)
        dis = get_cell_dis(self.sat_cell,dst_cell)
        if dis > 1:
            if dst not in self.route_table:
                try:
                    seg_addresses = pkt[ICMPv6TimeExceeded][IPv6ExtHdrSegmentRouting].addresses
                except Exception as e:
                    pass
                if seg_addresses[-1]!="a::a":
                    for i in range(len(seg_addresses)-1):
                        seg_cell = ip2cell(seg_addresses[i])
                        if seg_cell == self.sat_cell:
                            left_cells = seg_addresses[i+1:]
                            segs = ",".join(reversed(left_cells))
                            self.iproute.route("add",
                                table=self.table_id,
                                dst=dst, 
                                encap={"type": "seg6", "mode": "inline", "segs": segs},
                                oif=out_iface,
                                priority = 1)   
                            self.route_table.add(dst)
                            break
            
        if ICMPv6TimeExceeded in pkt:
            inner_ipv6 = pkt[ICMPv6TimeExceeded]
            if IPv6ExtHdrSegmentRouting in inner_ipv6:
                if pkt[ICMPv6TimeExceeded].segleft == 0:
                    packet.accept()
                    return
                pkt[ICMPv6TimeExceeded].dst = inner_ipv6[IPv6ExtHdrSegmentRouting].addresses[0]
                del pkt[ICMPv6TimeExceeded].cksum 
                packet.set_payload(bytes(pkt))
        packet.accept()
        return

if __name__ == "__main__":
    hostname = subprocess.check_output("hostname", shell=True).decode().strip()
    print(f"Current hostname: {hostname}")
    if hostname.startswith("GS"):
        RouteManager = GsRouteManager(hostname)
    elif hostname.startswith("SH"):
        RouteManager = SatRouteManager(hostname)
