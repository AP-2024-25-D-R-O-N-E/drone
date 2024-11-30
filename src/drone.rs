use std::collections::HashMap;

use crossbeam::{
    channel::{select_biased, Receiver, Sender},
    select,
};
use rand::seq::index;
use wg_2024::{
    controller::{DroneCommand, NodeEvent},
    network::NodeId,
    packet::{Nack, NackType, Packet},
};
use wg_2024::{drone::Drone as DroneTrait, packet::PacketType};

#[derive(Debug)]
pub struct MyDrone {
    drone_id: NodeId,
    sim_contr_send: Sender<NodeEvent>,            //Not packet.
    sim_contr_recv: Receiver<DroneCommand>,       //Not packet.
    packet_send: HashMap<NodeId, Sender<Packet>>, //All the sender to other nodes.
    packet_recv: Receiver<Packet>, //This drone receiver, that will be linked to a sender given to every other drone.
    pdr: u8,                       //Would keep it in % to occupy less space, but could be f32.
    floods_tracker: HashMap<NodeId, u64>, //not in the specification yet but will likely be needed to handle the Network Discovery Protocol
}

impl DroneTrait for MyDrone {
    fn new(options: wg_2024::drone::DroneOptions) -> Self {
        MyDrone {
            drone_id: options.id,
            sim_contr_send: options.controller_send,
            sim_contr_recv: options.controller_recv,
            packet_send: options.packet_send,
            packet_recv: options.packet_recv,
            pdr: (options.pdr * 100.0) as u8,
            floods_tracker: HashMap::new(),
        }
    }

    fn run(&mut self) {
        loop {
            select_biased! {
                //give priority to commands
                recv(self.sim_contr_recv) -> command_res => {
                    if let Ok(command) = command_res {
                        //handle the simulation controller's command
                    }
                },
                recv(self.packet_recv) -> packet_res => {
                    if let Ok(packet) = packet_res {
                    // each match branch may call a function to handle it to make it more readable

                        //temporary and just for testing
                        println!("packet received from {}, Packet: {:?}, {:?}", self.drone_id, packet.session_id, packet.pack_type);

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
                        match &packet.pack_type {
                            PacketType::Nack(_nack)=>self.forward_packet(packet),
                            PacketType::Ack(_ack)=>self.forward_packet(packet),
                            PacketType::MsgFragment(fragment)=>self.handle_msg_fragment(fragment.fragment_index, packet),
                            PacketType::FloodRequest(flood_request) => self.forward_packet(packet),
                            PacketType::FloodResponse(flood_response) => self.forward_packet(packet),
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
            println!(
                "packet sent from {}, Packet: {:?}, {:?}",
                self.drone_id, packet.session_id, packet.pack_type
            );

            let res = send_channel.send(packet);

            if let Err(packet) = res {
                //means there was an error with the channel (crashed drone), should spawn error
                self.packet_send.remove(&next_node);

                self.create_nack(
                    Self::error_index(&packet.0),
                    packet.0,
                    NackType::DestinationIsDrone,
                );
            }
        } else {
            //this means the channel isn't connected, should spawn error
            self.create_nack(
                Self::error_index(&packet),
                packet,
                NackType::DestinationIsDrone,
            );
        }
    }

    fn handle_msg_fragment(&mut self, index: u64, packet: Packet) {
        use rand::Rng;

        let prob: u8 = rand::thread_rng().gen_range(0..100);
        if prob < self.pdr {
            //reversing the route up to this point
            self.create_nack(index, packet, NackType::Dropped);
        } else {
            self.forward_packet(packet);
        }
    }

    fn create_nack(&mut self, index: u64, mut packet: Packet, nack_type: NackType) {
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
}
