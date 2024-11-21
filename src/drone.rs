use std::collections::HashMap;

use crossbeam::{
    channel::{Receiver, Sender},
    select,
};
use wg_2024::{controller::Command, network::NodeId, packet::Packet};
use wg_2024::{drone::Drone as DroneTrait, packet::PacketType};

#[derive(Debug)]
pub struct MyDrone {
    drone_id: NodeId,
    sim_contr_send: Sender<Command>,              //Not packet.
    sim_contr_recv: Receiver<Command>,            //Not packet.
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
                        println!("received packet at drone {}", self.drone_id);

                        //remember to remove the underscores when you actually start using the variable ig
                        match &packet.pack_type {
                            PacketType::Nack(_nack)=>self.forward_packet(packet),
                            PacketType::Ack(_ack)=>self.forward_packet(packet),
                            PacketType::MsgFragment(_fragment)=>self.forward_packet(packet),
                            PacketType::FloodRequest(flood_request) => self.forward_packet(packet),
                            PacketType::FloodResponse(flood_response) => self.forward_packet(packet),
                        }
                    }
                },
                recv(self.sim_contr_recv) -> command_res => {
                    if let Ok(command) = command_res {
                        //handle the simulation controller's command
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
        scs: Sender<Command>,
        scr: Receiver<Command>,
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
            let res = send_channel.send(packet);

            if res.is_err() {
                //means there was an error with the channel (crashed drone), should spawn error
            }

            // just testing
            println!("packet sent from {}", self.drone_id);
        } else {
            //this means the channel isn't connected, should spawn error
        }
    }
}
