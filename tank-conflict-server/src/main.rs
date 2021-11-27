use std::collections::{HashMap, HashSet};

use argh::FromArgs;
use log::info;
use message_io::{
    network::{Endpoint, Transport},
    node,
};
use rand::Rng;
use tank_conflict_server::{
    maze::{random_maze, MazeMap},
    protocol::{DataPacket, ServerDataPacket},
};
use uuid::Uuid;

#[derive(FromArgs)]
/// Options with which server will start
struct ServerOption {
    /// server port, 14582 by default
    #[argh(option, default = "14582")]
    port: u16,
    /// map width, 6 by default
    #[argh(option, default = "6")]
    map_x: u16,
    /// map height, 5 by default
    #[argh(option, default = "5")]
    map_y: u16,
}

#[derive(PartialEq, Eq)]
enum State {
    Spectating,
    Playing,
    Destroyed,
}

struct Player {
    uuid: Uuid,
    name: String,
    color: [f32; 3],
    score: u32,
    endpoint: Endpoint,
    state: State,
}

struct Server {
    players: HashMap<Uuid, Player>,
    livings: u32,
    map: MazeMap,
}

impl Server {
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            players: HashMap::new(),
            livings: 0,
            map: random_maze(width, height),
        }
    }

    pub fn regen_maze(&mut self, width: u32, height: u32) {
        self.map = random_maze(width, height);
    }
}

fn main() -> anyhow::Result<()> {
    env_logger::builder()
        .filter_level(log::LevelFilter::Warn)
        .filter_module("tank-conflict-server", log::LevelFilter::Info)
        .init();

    let opts: ServerOption = argh::from_env();

    info!("Server starting on port: {}", opts.port);

    let (handler, listner) = node::split::<()>();
    handler
        .network()
        .listen(Transport::FramedTcp, ("0.0.0.0", opts.port))?;

    info!("Generating random maze...");

    let mut server = Server::new(opts.map_x as _, opts.map_y as _);

    listner.for_each(|event| match event {
        node::NodeEvent::Network(event) => match event {
            message_io::network::NetEvent::Connected(_, _) => (),
            message_io::network::NetEvent::Accepted(ep, _) => info!("Endpoint connected: {:?}", ep),
            message_io::network::NetEvent::Message(ep, data) => {
                let data_packet: DataPacket = serde_json::from_slice(data).unwrap();
                if let DataPacket::C2S(uuid, packet) = data_packet {
                    match packet {
                        tank_conflict_server::protocol::ClientDataPacket::AccessServer {
                            color,
                            nickname,
                            uuid,
                        } => {
                            server.players.insert(
                                uuid,
                                Player {
                                    uuid,
                                    name: nickname.clone(),
                                    color,
                                    endpoint: ep,
                                    score: 0,
                                    state: State::Spectating,
                                },
                            );
                            let data_pack = DataPacket::S2C(ServerDataPacket::ServerState {
                                scores: server
                                    .players
                                    .iter()
                                    .map(|(uuid, pl)| (*uuid, pl.score))
                                    .collect::<HashMap<_, _>>(),
                                players: server
                                    .players
                                    .iter()
                                    .map(|(uuid, pl)| (*uuid, (pl.name.clone(), pl.color)))
                                    .collect::<HashMap<_, _>>(),
                            });
                            handler
                                .network()
                                .send(ep, &serde_json::to_vec(&data_pack).unwrap());
                            let data_pack = DataPacket::S2C(ServerDataPacket::NewPlayer {
                                color,
                                nickname,
                                uuid,
                            });
                            server.players.iter().for_each(|(_, pl)| {
                                handler
                                    .network()
                                    .send(pl.endpoint, &serde_json::to_vec(&data_pack).unwrap());
                            });
                            if server.players.len() == 2 {
                                let mut pluid = server.players.keys();
                                let spawns = vec![
                                    (*pluid.next().unwrap(), (0, 0)),
                                    (*pluid.next().unwrap(), (opts.map_x as _, opts.map_y as _)),
                                ];
                                server.regen_maze(opts.map_x as _, opts.map_y as _);
                                let data_pack = DataPacket::S2C(ServerDataPacket::NewRound {
                                    map: server.map.clone(),
                                    spawns,
                                });
                                server.players.iter().for_each(|(_, pl)| {
                                    handler.network().send(
                                        pl.endpoint,
                                        &serde_json::to_vec(&data_pack).unwrap(),
                                    );
                                });
                            }
                        }
                        tank_conflict_server::protocol::ClientDataPacket::TankPose {
                            position,
                            angle,
                        } => {
                            let data_pack = DataPacket::S2C(ServerDataPacket::TankPose {
                                uuid,
                                position,
                                angle,
                            });
                            server.players.iter().for_each(|(_, pl)| {
                                handler
                                    .network()
                                    .send(pl.endpoint, &serde_json::to_vec(&data_pack).unwrap());
                            });
                        }
                        tank_conflict_server::protocol::ClientDataPacket::BulletsFired {
                            position,
                            velocity,
                        } => {
                            let data_pack = DataPacket::S2C(ServerDataPacket::BulletsFired {
                                uuid,
                                position,
                                velocity,
                            });
                            server.players.iter().for_each(|(_, pl)| {
                                handler
                                    .network()
                                    .send(pl.endpoint, &serde_json::to_vec(&data_pack).unwrap());
                            });
                        }
                        tank_conflict_server::protocol::ClientDataPacket::Destroyed => {
                            let data_pack = DataPacket::S2C(ServerDataPacket::Destroyed(uuid));
                            server.players.iter().for_each(|(_, pl)| {
                                handler
                                    .network()
                                    .send(pl.endpoint, &serde_json::to_vec(&data_pack).unwrap());
                            });
                            server
                                .players
                                .entry(uuid)
                                .and_modify(|pl| pl.state = State::Destroyed);
                            server.livings -= 1;
                            if server.livings <= 1 {
                                let winner = server
                                    .players
                                    .iter()
                                    .find(|(_, pl)| pl.state == State::Playing);
                                if let Some((uuid, _)) = winner {
                                    let data_pack = DataPacket::S2C(ServerDataPacket::Score(*uuid));
                                    server.players.iter().for_each(|(_, pl)| {
                                        handler.network().send(
                                            pl.endpoint,
                                            &serde_json::to_vec(&data_pack).unwrap(),
                                        );
                                    });
                                }
                                let mut used_spawn = HashSet::new();
                                let spawns = server
                                    .players
                                    .iter()
                                    .map(|(uuid, _)| {
                                        let mut rng = rand::thread_rng();
                                        let (mut x, mut y) = (
                                            rng.gen_range(0..opts.map_x),
                                            rng.gen_range(0..opts.map_y),
                                        );
                                        while used_spawn.contains(&(x, y)) {
                                            x = rng.gen_range(0..opts.map_x);
                                            y = rng.gen_range(0..opts.map_y);
                                        }
                                        used_spawn.insert((x, y));
                                        (*uuid, (x as u32, y as u32))
                                    })
                                    .collect::<Vec<_>>();
                                server.regen_maze(opts.map_x as _, opts.map_y as _);
                                let data_pack = DataPacket::S2C(ServerDataPacket::NewRound {
                                    map: server.map.clone(),
                                    spawns,
                                });
                                server.players.iter().for_each(|(_, pl)| {
                                    handler.network().send(
                                        pl.endpoint,
                                        &serde_json::to_vec(&data_pack).unwrap(),
                                    );
                                });
                            }
                        }
                    }
                }
            }
            message_io::network::NetEvent::Disconnected(ep) => {
                let mut quitter_uuid = Uuid::default();
                server.players.retain(|uuid, pl| {
                    if pl.endpoint == ep {
                        info!("Player {}/{} quitted!", pl.name, pl.uuid);
                        quitter_uuid = *uuid;
                        return false;
                    } else {
                        return true;
                    }
                });
                let data_pack = DataPacket::S2C(ServerDataPacket::Quit(quitter_uuid));
                server.players.iter().for_each(|(_, pl)| {
                    handler
                        .network()
                        .send(pl.endpoint, &serde_json::to_vec(&data_pack).unwrap());
                });
            }
        },
        _ => (),
    });

    Ok(())
}
