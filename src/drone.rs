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
    in_crash_behaviour: bool,
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
            in_crash_behaviour: false,
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
                            DroneCommand::Crash=> {
                                self.in_crash_behaviour = true;
                            },
                            DroneCommand::RemoveSender(id) => self.remove_channel(id), }
                    }
                },
                recv(self.packet_recv) -> packet_res => {

                    match packet_res {
                        Ok(mut packet) => {
                            // each match branch may call a function to handle it to make it more readable

                                //temporary and just for testing
                                log::debug!("{} at {} - packet: {}", " <- packet received".green(), self.drone_id, packet);

                                // flood_request works very differently from the other types as it's a broadcast, as such it's handled on its own
                                if let PacketType::FloodRequest(request) = &packet.pack_type {
                                    // only requirement for flood_request is that the destination not be a weird
                                    if let Err(nack_type) = self.check_recipient(&packet) {
                                        // if there's an issue with the flood request formatting, create and send back a nack
                                        self.create_nack(0, &mut packet, nack_type);
                                        self.forward_packet(packet);
                                    } else {
                                        // if there's no recipient error, handle flood_requests normally
                                        self.forward_flood_request(packet);
                                    }
                                } else {
                                    // handle packets that aren't flood requests

                                    // if there's an error, create a nack
                                    if let Err(nack_type) = self.check_handling_errors(&packet) {

                                        // if the error occurs on Ack, Nack or FloodResponse, we send it back through the simulation controller
                                        match &packet.pack_type {
                                            PacketType::Ack(_) |
                                            PacketType::Nack(_) |
                                            PacketType::FloodResponse(_) => {
                                                self.sim_contr_send.send(DroneEvent::ControllerShortcut(packet));
                                                // we don't want to forward any packet in this case since we used the shortcut, so we continue to the next recv
                                                continue;
                                            },
                                            PacketType::MsgFragment(fragment) => {
                                                // if the packet is a msgfragment, then create an appropriate nack
                                                self.create_nack(fragment.fragment_index, &mut packet, nack_type);
                                            },
                                            _ => {
                                                // flood requests shouldn't reach here
                                                continue;
                                            }
                                        }
                                    }

                                    self.forward_packet(packet);

                                }

                            },
                        Err(e) => {
                            // in theory errors only happen when the channel has no open senders
                            if self.in_crash_behaviour {
                                return;
                            }
                        },
                    }
                }
            }
        }
    }
}

impl MyDrone {
    fn check_handling_errors(&mut self, packet: &Packet) -> Result<(), (NackType)> {
        // checks if this drone is the recipient
        self.check_recipient(packet)?;

        // checks if drone is the final destination of the packet
        self.check_final_dest(packet)?;

        // checks if the next packet is reachable from here
        self.check_neighbor_recipient(packet)?;

        // if the packet is a fragment, calculate the drop rate
        if let PacketType::MsgFragment(fragment) = &packet.pack_type {
            self.check_fragment_drop(packet)?;
        }

        Ok(())
    }

    fn check_recipient(&self, packet: &Packet) -> Result<(), (NackType)> {
        // check if the recipient of the packet is correct
        let node_id = packet.routing_header.hops[packet.routing_header.hop_index];

        if node_id != self.drone_id {
            Err(NackType::UnexpectedRecipient(node_id))
        } else {
            Ok(())
        }
    }

    fn check_final_dest(&self, packet: &Packet) -> Result<(), (NackType)> {
        // if the hop_index is the last of the hops vector, then the destination is a drone, which isn't allowed
        if packet.routing_header.hops.len() - 1 == packet.routing_header.hop_index {
            Err(NackType::DestinationIsDrone)
        } else {
            Ok(())
        }
    }

    fn check_neighbor_recipient(&self, packet: &Packet) -> Result<(), (NackType)> {
        // since this check is done after check_final_dest, we know that hop_index isn't the last item of the vector
        let node_id = packet.routing_header.hops[packet.routing_header.hop_index + 1];

        // we assume that packet_send containing the hashmap ensures the fact that the sender channel will work
        if self.packet_send.contains_key(&node_id) {
            Ok(())
        } else {
            Err(NackType::ErrorInRouting(node_id))
        }
    }

    fn check_fragment_drop(&self, packet: &Packet) -> Result<(), (NackType)> {
        use rand::Rng;

        let prob: f32 = rand::thread_rng().gen_range(0.0..1.0);
        if prob <= self.pdr {
            // if the packet gets dropped, send an event to the simulation controller
            self.sim_contr_send
                .send(DroneEvent::PacketDropped(packet.clone()));
            Err(NackType::Dropped)
        } else {
            Ok(())
        }
    }

    fn forward_packet(&mut self, mut packet: Packet) {
        packet.routing_header.hop_index += 1;
        let next_node = packet.routing_header.hops[packet.routing_header.hop_index];

        let send_channel = &self.packet_send[&next_node];

        log::debug!(
            "{} from {} - packet: {}",
            " -> packet sent ".blue(),
            self.drone_id,
            packet
        );

        let res = send_channel.send(packet.clone());

        if let Err(mut packet) = res {
            log::error!("The send inside channel gave an error, this shouldn't be happening");
        } else {
            self.sim_contr_send.send(DroneEvent::PacketSent(packet));
        }
    }

    fn forward_flood_request(&mut self, mut packet: Packet) {
        // let mut flood_request: FloodRequest;
        if let PacketType::FloodRequest(mut flood_request) = packet.pack_type {
            // push node into path_trace, this assures that every edge can be reconstructed by the initiator node
            flood_request
                .path_trace
                .push((self.drone_id, NodeType::Drone));

                //insert returns whether the element was already present in the set
            if !self.floods_tracker
                .insert((flood_request.initiator_id, flood_request.flood_id)) || self.packet_send.len() == 1
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
    fn create_nack(&self, index: u64, packet: &mut Packet, nack_type: NackType) {
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
            nack_type,
        };

        packet.pack_type = PacketType::Nack(nack);
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

#[test]
fn test() {

}