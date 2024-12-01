use std::collections::HashMap;

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
    floods_tracker: HashMap<NodeId, u64>, //not in the specification yet but will likely be needed to handle the Network Discovery Protocol
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
                            PacketType::FloodRequest(flood_request) => self.forward_packet(packet),
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
            floods_tracker: HashMap::new(),
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

        println!("PROVA LOCAL");
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

    // fn forward_discovery_query(&self, mut flood_request: FloodRequest, packet: Packet) {
    //     // packet: Packet
    //     // the packet.packet_type was matched for PacketType::Query
    //     // query: Query
    //
    //     // push node into path_trace, this assures that every edge can be reconstructed by the initiator node
    //     flood_request
    //         .path_trace
    //         .push((self.drone_id, NodeType::Drone));
    //
    //     if (flood_request.ttl == 0) {
    //         //only send back a result if the flooding is not a broadcast
    //         if !flood_request.broadcasting {
    //             let query_result = QueryResult {
    //                 path_trace: flood_request.path_trace.clone(),
    //                 flood_id: flood_request.flood_id,
    //             };
    //
    //             let return_routing: Vec<u64> = flood_request
    //                 .path_trace
    //                 .iter()
    //                 .rev()
    //                 .map(|(node_id, _node_type)| *node_id)
    //                 .collect();
    //
    //             let packet = Packet {
    //                 pack_type: PacketType::QueryResult(query_result),
    //                 routing_header: SourceRoutingHeader {
    //                     hops: return_routing.clone(),
    //                 },
    //                 session_id: 0, // it'll be whatever for now
    //             };
    //
    //             let first_hop_channel = self.neighbors.get(&return_routing[0]).unwrap();
    //             // unwrap since given the construction of the hops we're guaranteed to not have any errors
    //
    //             let _ = first_hop_channel.send(packet);
    //         }
    //     } else {
    //         //check if the flood was already managed on this node
    //
    //         let floodVal = self
    //             .floods_tracker
    //             .get(&flood_request.initiator_id)
    //             .or(Some(&0));
    //
    //         if let Some(&flood_id) = floodVal {
    //             if flood_id < flood_request.flood_id {
    //                 // this means that the query's flood id is a new flood, thus it needs to be processed
    //
    //                 flood_request.ttl -= 1;
    //
    //                 for (neighbor_id, neighbor_channel) in &self.neighbors {
    //                     let mut route = packet.routing_header.hops.clone();
    //
    //                     route.push(*neighbor_id);
    //
    //                     let packet = Packet {
    //                         pack_type: PacketType::Query(flood_request.clone()),
    //                         routing_header: SourceRoutingHeader { hops: route },
    //                         session_id: 0, // it'll be whatever for now
    //                     };
    //                 }
    //             } else {
    //                 //flood's already passed through this node, thus send back a QueryResult
    //
    //                 //only send back a result if the flooding is not a broadcast
    //                 if !flood_request.broadcasting {
    //                     let query_result = QueryResult {
    //                         path_trace: flood_request.path_trace.clone(),
    //                         flood_id: flood_request.flood_id,
    //                     };
    //
    //                     let mut return_routing: Vec<u64> = flood_request
    //                         .path_trace
    //                         .iter()
    //                         .map_while(|(node_id, _node_type)| {
    //                             if *node_id != self.id {
    //                                 Some(*node_id)
    //                             } else {
    //                                 None
    //                             }
    //                         })
    //                         .collect();
    //
    //                     return_routing.reverse();
    //
    //                     let packet = Packet {
    //                         pack_type: PacketType::QueryResult(query_result),
    //                         routing_header: SourceRoutingHeader {
    //                             hops: return_routing.clone(),
    //                         },
    //                         session_id: 0, // it'll be whatever for now
    //                     };
    //
    //                     let first_hop_channel = self.neighbors.get(&return_routing[0]).unwrap();
    //                     // unwrap since given the construction of the hops we're guaranteed to not have any errors
    //
    //                     let _ = first_hop_channel.send(packet);
    //                 }
    //             }
    //         }
    //     }
    // }
}
