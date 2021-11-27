pub mod maze {
    use std::collections::VecDeque;

    use rand::Rng;
    use serde::{Deserialize, Serialize};

    bitflags::bitflags! {
        #[derive(Serialize, Deserialize)]
        pub struct BlockWall: u32 {
            const TOP = 0b00000001;
            const RIGHT = 0b00000010;
            const BOTTOM = 0b00000100;
            const LEFT = 0b00001000;
        }
    }

    pub type MazeMap = Vec<Vec<BlockWall>>;

    fn paint_region(
        point: (usize, usize),
        dimension: (usize, usize),
        painting: &mut Vec<Vec<bool>>,
        walls: &MazeMap,
    ) {
        let mut point_queue: VecDeque<(usize, usize)> = VecDeque::new();
        point_queue.push_back(point);
        while point_queue.len() > 0 {
            let (now_x, now_y) = point_queue.pop_front().unwrap();
            painting[now_x][now_y] = true;
            if now_x > 0 {
                if !painting[now_x - 1][now_y]
                    && !walls[now_x][now_y].contains(BlockWall::LEFT)
                    && !walls[now_x - 1][now_y].contains(BlockWall::RIGHT)
                {
                    point_queue.push_back((now_x - 1, now_y));
                }
            }
            if now_x < dimension.0 - 1 {
                if !painting[now_x + 1][now_y]
                    && !walls[now_x][now_y].contains(BlockWall::RIGHT)
                    && !walls[now_x + 1][now_y].contains(BlockWall::LEFT)
                {
                    point_queue.push_back((now_x + 1, now_y));
                }
            }
            if now_y > 0 {
                if !painting[now_x][now_y - 1]
                    && !walls[now_x][now_y].contains(BlockWall::TOP)
                    && !walls[now_x][now_y - 1].contains(BlockWall::BOTTOM)
                {
                    point_queue.push_back((now_x, now_y - 1));
                }
            }
            if now_y < dimension.1 - 1 {
                if !painting[now_x][now_y + 1]
                    && !walls[now_x][now_y].contains(BlockWall::BOTTOM)
                    && !walls[now_x][now_y + 1].contains(BlockWall::TOP)
                {
                    point_queue.push_back((now_x, now_y + 1));
                }
            }
        }
    }

    pub fn random_maze(width: u32, height: u32) -> MazeMap {
        let mut wall_map = vec![vec![BlockWall::empty(); height as usize]; width as usize];
        let mut rng = rand::thread_rng();
        wall_map.iter_mut().enumerate().for_each(|(x, column)| {
            column.iter_mut().enumerate().for_each(|(y, block)| {
                if x == 0 {
                    *block |= BlockWall::LEFT;
                }
                if x == (width - 1) as usize {
                    *block |= BlockWall::RIGHT;
                }
                if y == 0 {
                    *block |= BlockWall::TOP;
                }
                if y == (height - 1) as usize {
                    *block |= BlockWall::BOTTOM;
                }
                if rng.gen_bool(0.44) {
                    *block |= BlockWall::LEFT;
                }
                if rng.gen_bool(0.44) {
                    *block |= BlockWall::RIGHT;
                }
                if rng.gen_bool(0.44) {
                    *block |= BlockWall::TOP;
                }
                if rng.gen_bool(0.44) {
                    *block |= BlockWall::BOTTOM;
                }
            })
        });
        //punch through
        let mut painting = vec![vec![false; height as usize]; width as usize];
        let mut visited = vec![vec![false; height as usize]; width as usize];
        let mut point_queue: VecDeque<(usize, usize)> = VecDeque::new();
        let initial = (rng.gen_range(0..width) as _, rng.gen_range(0..height) as _);
        paint_region(initial, (width as _, height as _), &mut painting, &wall_map);
        point_queue.push_back(initial);
        while point_queue.len() > 0 {
            let (now_x, now_y) = point_queue.pop_front().unwrap();
            painting[now_x][now_y] = true;
            visited[now_x][now_y] = true;
            if now_x > 0 {
                if !painting[now_x - 1][now_y]
                    && (wall_map[now_x][now_y].contains(BlockWall::LEFT)
                        || wall_map[now_x - 1][now_y].contains(BlockWall::RIGHT))
                {
                    wall_map[now_x][now_y].remove(BlockWall::LEFT);
                    wall_map[now_x - 1][now_y].remove(BlockWall::RIGHT);
                    paint_region(
                        (now_x - 1, now_y),
                        (width as _, height as _),
                        &mut painting,
                        &wall_map,
                    );
                    point_queue.push_back((now_x - 1, now_y));
                } else if !visited[now_x - 1][now_y]
                    && !wall_map[now_x][now_y].contains(BlockWall::LEFT)
                    && !wall_map[now_x - 1][now_y].contains(BlockWall::RIGHT)
                {
                    point_queue.push_back((now_x - 1, now_y));
                }
            }
            if now_x < (width - 1) as usize {
                if !painting[now_x + 1][now_y]
                    && (wall_map[now_x][now_y].contains(BlockWall::RIGHT)
                        || wall_map[now_x + 1][now_y].contains(BlockWall::LEFT))
                {
                    wall_map[now_x][now_y].remove(BlockWall::RIGHT);
                    wall_map[now_x + 1][now_y].remove(BlockWall::LEFT);
                    paint_region(
                        (now_x + 1, now_y),
                        (width as _, height as _),
                        &mut painting,
                        &wall_map,
                    );
                    point_queue.push_back((now_x + 1, now_y));
                } else if !visited[now_x + 1][now_y]
                    && !wall_map[now_x][now_y].contains(BlockWall::RIGHT)
                    && !wall_map[now_x + 1][now_y].contains(BlockWall::LEFT)
                {
                    point_queue.push_back((now_x + 1, now_y));
                }
            }
            if now_y > 0 {
                if !painting[now_x][now_y - 1]
                    && (wall_map[now_x][now_y].contains(BlockWall::TOP)
                        || wall_map[now_x][now_y - 1].contains(BlockWall::BOTTOM))
                {
                    wall_map[now_x][now_y].remove(BlockWall::TOP);
                    wall_map[now_x][now_y - 1].remove(BlockWall::BOTTOM);
                    paint_region(
                        (now_x, now_y - 1),
                        (width as _, height as _),
                        &mut painting,
                        &wall_map,
                    );
                    point_queue.push_back((now_x, now_y - 1));
                } else if !visited[now_x][now_y - 1]
                    && !wall_map[now_x][now_y].contains(BlockWall::TOP)
                    && !wall_map[now_x][now_y - 1].contains(BlockWall::BOTTOM)
                {
                    point_queue.push_back((now_x, now_y - 1));
                }
            }
            if now_y < (height - 1) as usize {
                if !painting[now_x][now_y + 1]
                    && (wall_map[now_x][now_y].contains(BlockWall::BOTTOM)
                        || wall_map[now_x][now_y + 1].contains(BlockWall::TOP))
                {
                    wall_map[now_x][now_y].remove(BlockWall::BOTTOM);
                    wall_map[now_x][now_y + 1].remove(BlockWall::TOP);
                    paint_region(
                        (now_x, now_y + 1),
                        (width as _, height as _),
                        &mut painting,
                        &wall_map,
                    );
                    point_queue.push_back((now_x, now_y + 1));
                } else if !visited[now_x][now_y + 1]
                    && !wall_map[now_x][now_y].contains(BlockWall::BOTTOM)
                    && !wall_map[now_x][now_y + 1].contains(BlockWall::TOP)
                {
                    point_queue.push_back((now_x, now_y + 1));
                }
            }
        }
        return wall_map;
    }
}

pub mod protocol {
    use std::collections::HashMap;

    use serde::{Deserialize, Serialize};
    use uuid::Uuid;

    use crate::maze::MazeMap;

    #[derive(Clone, Debug, Serialize, Deserialize)]
    pub enum ClientDataPacket {
        AccessServer {
            color: [f32; 3],
            nickname: String,
            uuid: Uuid,
        },
        TankPose {
            position: [f32; 2],
            angle: f32,
        },
        BulletsFired {
            position: [f32; 2],
            velocity: [f32; 2],
        },
        Destroyed,
    }

    #[derive(Clone, Debug, Serialize, Deserialize)]
    pub enum ServerDataPacket {
        NewPlayer {
            color: [f32; 3],
            nickname: String,
            uuid: Uuid,
        },
        Destroyed(Uuid),
        TankPose {
            uuid: Uuid,
            position: [f32; 2],
            angle: f32,
        },
        BulletsFired {
            uuid: Uuid,
            position: [f32; 2],
            velocity: [f32; 2],
        },
        ServerState {
            scores: HashMap<Uuid, u32>,
            players: HashMap<Uuid, (String, [f32; 3])>,
        },
        NewRound {
            map: MazeMap,
            spawns: Vec<(Uuid, (u32, u32))>,
        },
        Score(Uuid),
        Quit(Uuid),
    }

    #[derive(Clone, Debug, Serialize, Deserialize)]
    pub enum DataPacket {
        C2S(Uuid, ClientDataPacket),
        S2C(ServerDataPacket),
    }
}
