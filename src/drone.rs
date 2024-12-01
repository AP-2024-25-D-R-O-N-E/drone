use std::collections::{HashMap, HashSet};

use crossbeam::{
    channel::{Receiver, Sender},
    select,
};

use wg_2024::drone::Drone as DroneTrait;
use wg_2024::{controller::*, network::*, packet::*};

#[derive(Debug)]
pub struct MyDrone {
    drone_id: NodeId,
    sim_contr_send: Sender<DroneCommand>,         //Not packet.
    sim_contr_recv: Receiver<DroneCommand>,       //Not packet.
    packet_send: HashMap<NodeId, Sender<Packet>>, //All the sender to other nodes.
    packet_recv: Receiver<Packet>, //This drone receiver, that will be linked to a sender given to every other drone.
    pdr: u8,                       //Would keep it in % to occupy less space, but could be f32.
    floods_tracker: HashSet<(NodeId, u64)>, //not in the specification yet but will likely be needed to handle the Network Discovery Protocol
}

impl DroneTrait for MyDrone {
    /// since this function is giving me massive ???? for missing the initialization of the sending packets, we won't use it ig
    fn new(options: wg_2024::drone::DroneOptions) -> Self {
        todo!()
    }

    fn run(&mut self) {
        loop {
            select! {
                recv(self.packet_recv) -> packet_res => {
                    if let Ok(packet) = packet_res {
                    // each match branch may call a function to handle it to make it more readable

                        //temporary and just for testing
                        println!("packet received from {}, Packet: {:?}, {:?}", self.drone_id, packet.session_id, packet.pack_type);

                        //remember to remove the underscores when you actually start using the variable ig
                        match &packet.pack_type {
                            PacketType::Nack(_nack)=>self.forward_packet(packet),
                            PacketType::Ack(_ack)=>self.forward_packet(packet),
                            PacketType::MsgFragment(fragment)=>self.handle_MsgFragment(fragment.fragment_index, packet),
                            PacketType::FloodRequest(flood_request) => self.forward_flood_request(flood_request.clone(), packet),
                            PacketType::FloodResponse(flood_response) => self.forward_packet(packet),
                        }


                    }
                },
                recv(self.sim_contr_recv) -> drone_command_res => {
                    if let Ok(drone_command) = drone_command_res {
                        //handle the simulation controller's DroneCommand
                    }
                }
            }
        }
    }
}

impl MyDrone {
    /// reminder that this is a temporary function until we sort out the send channels being implemented in the drone trait API
    pub fn new_drone(
        id: NodeId,
        scs: Sender<DroneCommand>,
        scr: Receiver<DroneCommand>,
        ps: HashMap<NodeId, Sender<Packet>>,
        pr: Receiver<Packet>,
        pdr: f32,
    ) -> MyDrone {
        MyDrone {
            drone_id: id,
            sim_contr_send: scs,
            sim_contr_recv: scr,
            packet_send: ps,
            packet_recv: pr,
            pdr: (pdr * 100.0) as u8,
            floods_tracker: HashSet::new(),
        }
    }

    fn forward_packet(&self, mut packet: Packet) {
        // go to next index
        if packet.routing_header.hop_index == packet.routing_header.hops.len() - 1 {
            //reached destination, technically this shouldn't be done here but client/server side
            return;
        }

        packet.routing_header.hop_index += 1;
        let next_node = packet.routing_header.hops[packet.routing_header.hop_index];

        let next_send = self.packet_send.get(&next_node);

        if let Some(send_channel) = next_send {
            println!(
                "packet sent from {}, Packet: {:?}, {:?}",
                self.drone_id, packet.session_id, packet.pack_type
            );

            let res = send_channel.send(packet);

            if res.is_err() {
                //means there was an error with the channel (crashed drone), should spawn error
            }

            // just testing
        } else {
            //this means the channel isn't connected, should spawn error
        }
    }

    fn handle_MsgFragment(&self, index: u64, mut packet: Packet) {
        use rand::Rng;

        let prob: u8 = rand::thread_rng().gen_range(0..100);
        if prob < self.pdr {
            //reversing the route up to this point
            packet
                .routing_header
                .hops
                .truncate(packet.routing_header.hop_index + 1);
            packet.routing_header.hops.reverse();
            packet.routing_header.hop_index = 0;

            //packet becomes Nack type and gets forwarded
            let nack = Nack {
                fragment_index: index,
                nack_type: NackType::Dropped,
            };

            packet.pack_type = PacketType::Nack(nack);
        }

        self.forward_packet(packet);
    }

    fn forward_flood_request(&mut self, mut flood_request: FloodRequest, packet: Packet) {
        // packet: Packet
        // the packet.packet_type was matched for PacketType::Query
        // query: Query

        // push node into path_trace, this assures that every edge can be reconstructed by the initiator node
        flood_request
            .path_trace
            .push((self.drone_id, NodeType::Drone));

        if self
            .floods_tracker
            .contains(&(flood_request.initiator_id, flood_request.flood_id))
        {
            //send flood response back, since node is already visited
            let new_flood_res = FloodResponse {
                path_trace: flood_request.path_trace.clone(),
                flood_id: flood_request.flood_id,
            };

            let mut inverse_route = packet.routing_header.hops.clone();
            inverse_route.truncate(packet.routing_header.hop_index + 1);
            inverse_route.reverse();

            let packet = Packet {
                pack_type: PacketType::FloodResponse(new_flood_res),
                routing_header: SourceRoutingHeader {
                    hops: inverse_route,
                    hop_index: 0,
                },
                session_id: 0, // it'll be whatever for now
            };

            self.forward_packet(packet);
        } else {
            // mark as visited and forward

            println!("Discovered drone {}", self.drone_id);
            self.floods_tracker
                .insert((flood_request.initiator_id, flood_request.flood_id));

            for (neighbor_id, neighbor_channel) in &self.packet_send {
                let prev_hop_index = packet.routing_header.hop_index - 1;
                if *neighbor_id != packet.routing_header.hops[prev_hop_index] {
                    // do not send to the node the request is coming from
                    let mut route = packet.routing_header.hops.clone();
                    route.push(*neighbor_id);

                    let mut new_flood_req = flood_request.clone();
                    new_flood_req
                        .path_trace
                        .push((*neighbor_id, NodeType::Drone));

                    let packet = Packet {
                        pack_type: PacketType::FloodRequest(new_flood_req),
                        routing_header: SourceRoutingHeader {
                            hops: route,
                            hop_index: packet.routing_header.hop_index,
                        },
                        session_id: 0, // it'll be whatever for now
                    };
                    self.forward_packet(packet);
                }
            }
        }
    }
}
