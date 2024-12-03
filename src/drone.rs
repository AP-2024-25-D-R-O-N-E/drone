use colored::Colorize;
use log::*;

use std::collections::{HashMap, HashSet};

use crossbeam::{
    channel::{select_biased, Receiver, Sender},
    select,
};
use rand::seq::index;
use wg_2024::{
    controller::{DroneCommand, DroneEvent},
    network::{NodeId, SourceRoutingHeader},
    packet::{FloodResponse, Nack, NackType, NodeType, Packet},
};
use wg_2024::{drone::Drone as DroneTrait, packet::PacketType};

#[derive(Debug)]
pub struct MyDrone {
    drone_id: NodeId,
    sim_contr_send: Sender<DroneEvent>,           //Not packet.
    sim_contr_recv: Receiver<DroneCommand>,       //Not packet.
    packet_send: HashMap<NodeId, Sender<Packet>>, //All the sender to other nodes.
    packet_recv: Receiver<Packet>, //This drone receiver, that will be linked to a sender given to every other drone.
    pdr: f32,                      //Would keep it in % to occupy less space, but could be f32.
    floods_tracker: HashSet<(NodeId, u64)>, //not in the specification yet but will likely be needed to handle the Network Discovery Protocol
}

impl DroneTrait for MyDrone {
    fn new(
        id: NodeId,
        controller_send: Sender<DroneEvent>,
        controller_recv: Receiver<DroneCommand>,
        packet_recv: Receiver<Packet>,
        packet_send: HashMap<NodeId, Sender<Packet>>,
        pdr: f32,
    ) -> Self {
        MyDrone {
            drone_id: id,
            sim_contr_send: controller_send,
            sim_contr_recv: controller_recv,
            packet_send,
            packet_recv,
            pdr,
            floods_tracker: HashSet::new(),
        }
    }

    fn run(&mut self) {
        loop {
            select_biased! {
                //give priority to commands
                recv(self.sim_contr_recv) -> command_res => {
                    if let Ok(command) = command_res {
                        //handle the simulation controller's command
                        match command {
                            DroneCommand::AddSender(id,sender)=>self.add_sender(id,sender),
                            DroneCommand::SetPacketDropRate(pdr)=>self.set_packet_droprate(pdr),
                            DroneCommand::Crash=>todo!(),
                            DroneCommand::RemoveSender(_) => todo!(), }
                    }
                },
                recv(self.packet_recv) -> packet_res => {
                    if let Ok(mut packet) = packet_res {
                    // each match branch may call a function to handle it to make it more readable

                        //temporary and just for testing
                        log::debug!("{} at {} - packet: {:?}, {:?}", " <- packet received".green(), self.drone_id, packet.session_id, packet.pack_type);

                        // error
                        if packet.routing_header.hops[packet.routing_header.hop_index] != self.drone_id {
                            // send back Nack with unexpected recipient

                            let node_id = packet.routing_header.hops[packet.routing_header.hop_index];

                            let mut index = 0;

                            if let PacketType::MsgFragment(fragment) = &packet.pack_type {
                                index = fragment.fragment_index;
                            }

                            self.create_nack(index, packet, NackType::UnexpectedRecipient(node_id));

                            //go to next loop
                            continue;
                        }

                        //remember to remove the underscores when you actually start using the variable ig
                        match &mut packet.pack_type {
                            PacketType::Nack(_nack)=>self.forward_packet(packet),
                            PacketType::Ack(_ack)=>self.forward_packet(packet),
                            PacketType::MsgFragment(fragment)=>self.handle_msg_fragment(fragment.fragment_index, packet),
                            PacketType::FloodRequest(_) => self.forward_flood_request( packet ),
                            PacketType::FloodResponse(_) => self.forward_packet(packet),
                        }


                    }
                }
            }
        }
    }
}

impl MyDrone {
    fn forward_packet(&mut self, mut packet: Packet) {
        //remember to send events to the simulation controller

        //remember to send events to the simulation controller

        // go to next index
        if packet.routing_header.hop_index == packet.routing_header.hops.len() - 1 {
            //reached destination error
            self.create_nack(
                Self::error_index(&packet),
                packet,
                NackType::DestinationIsDrone,
            );
            return;
        }

        packet.routing_header.hop_index += 1;
        let next_node = packet.routing_header.hops[packet.routing_header.hop_index];

        let next_send = self.packet_send.get(&next_node);

        if let Some(send_channel) = next_send {
            log::debug!(
                "{} from {} - packet: {:?}, {:?}",
                " -> packet sent ".blue(),
                self.drone_id,
                packet.session_id,
                packet.pack_type
            );

            let res = send_channel.send(packet);

            if let Err(packet) = res {
                //means there was an error with the channel (crashed drone), should spawn error
                self.packet_send.remove(&next_node);

                self.create_nack(
                    Self::error_index(&packet.0),
                    packet.0,
                    NackType::ErrorInRouting(next_node),
                );
            }
        } else {
            //this means the channel isn't connected, should spawn error
            self.create_nack(
                Self::error_index(&packet),
                packet,
                NackType::ErrorInRouting(next_node),
            );
        }
    }

    fn handle_msg_fragment(&mut self, index: u64, packet: Packet) {
        use rand::Rng;

        let prob: f32 = rand::thread_rng().gen_range(0.0..1.0);
        if prob <= self.pdr {
            //reversing the route up to this point
            self.create_nack(index, packet, NackType::Dropped);
        } else {
            self.forward_packet(packet);
        }
    }

    fn forward_flood_request(&mut self, mut packet: Packet) {
        // let mut flood_request: FloodRequest;
        if let PacketType::FloodRequest(mut flood_request) = packet.pack_type {
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
                    path_trace: flood_request.path_trace,
                    flood_id: flood_request.flood_id,
                };

                let mut inverse_route = packet.routing_header.hops;
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

                log::info!("Discovered drone {}", self.drone_id);
                self.floods_tracker
                    .insert((flood_request.initiator_id, flood_request.flood_id));

                let neighbors: Vec<_> = self.packet_send.keys().cloned().collect(); // should tecnically avoid cloning the crossbeam channel
                for neighbor_id in neighbors {
                    let prev_hop_index = packet.routing_header.hop_index - 1;
                    if neighbor_id != packet.routing_header.hops[prev_hop_index] {
                        // do not send to the node the request is coming from
                        let mut route = packet.routing_header.hops.clone();
                        route.push(neighbor_id);

                        let packet = Packet {
                            pack_type: PacketType::FloodRequest(flood_request.clone()),
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
    fn create_nack(&self, index: u64, mut packet: Packet, nack_type: NackType) {
        //reversing the route up to this point
        packet
            .routing_header
            .hops
            .truncate(packet.routing_header.hop_index + 1);
        packet.routing_header.hops.reverse();
        packet.routing_header.hop_index = 1;

        // might need to do tests here too?

        //packet becomes Nack type and gets forwarded
        let nack = Nack {
            fragment_index: index,
            nack_type,
        };

        packet.pack_type = PacketType::Nack(nack);

        let next_send = self.packet_send.get(&packet.routing_header.hops[1]);

        if let Some(channel) = next_send {
            let res = channel.send(packet);

            if res.is_err() {
                //means that the message got "trapped"
                // once behaviour is defined it should be something along the lines of "tell the simulation controller"
            }
        }
    }

    fn error_index(packet: &Packet) -> u64 {
        let mut index = 0;
        if let PacketType::MsgFragment(fragment) = &packet.pack_type {
            index = fragment.fragment_index;
        }
        index
    }

    fn add_sender(&mut self, id: NodeId, sender: Sender<Packet>) {
        self.packet_send.insert(id, sender);
    }

    fn set_packet_droprate(&mut self, pdr: f32) {
        self.pdr = pdr;
    }

    fn remove_channel(&mut self, id: NodeId) {
        self.packet_send.remove(&id);
    }
}
