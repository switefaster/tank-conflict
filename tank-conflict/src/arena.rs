use std::collections::VecDeque;

use image::GenericImageView;
use rapier2d::prelude::*;
use tank_conflict_server::maze::{BlockWall, MazeMap};
use wgpu::util::DeviceExt;

use crate::{Data, Vertex};

#[derive(Clone)]
pub struct WallBox {
    extent: cgmath::Vector2<f32>,
    center: cgmath::Point2<f32>,
    rotation: cgmath::Rad<f32>,
}

const WALL_LENGTH: u32 = 110;
const WALL_WIDTH: u32 = 10;

pub struct Arena {
    vertices: wgpu::Buffer,
    vertices_count: u32,
    transform_bind_group: wgpu::BindGroup,
    asset_bind_group: wgpu::BindGroup,
}

impl Arena {
    pub fn new(
        wall_map: MazeMap,
        transform_layout: &wgpu::BindGroupLayout,
        asset_layout: &wgpu::BindGroupLayout,
        sampler: &wgpu::Sampler,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
    ) -> Self {
        //generate mesh
        let start_x = width as f32 * -0.5 * WALL_LENGTH as f32;
        let start_y = height as f32 * 0.5 * WALL_LENGTH as f32;
        let half_length = WALL_LENGTH as f32 * 0.5;
        let half_width = WALL_WIDTH as f32 * 0.5;
        let mut vertices_data = Vec::new();
        let mut colliders = Vec::new();
        for x in 0..width {
            for y in 0..height {
                let center_x = start_x + (x * WALL_LENGTH) as f32;
                let center_y = start_y - (y * WALL_LENGTH) as f32;
                let x = x as usize;
                let y = y as usize;
                let mut horiz_boxes = vec![];
                let mut vert_boxes = vec![];
                if wall_map[x][y].contains(BlockWall::TOP) {
                    if y == 0 || (y > 0 && !wall_map[x][y - 1].contains(BlockWall::BOTTOM)) {
                        horiz_boxes.push(WallBox {
                            extent: cgmath::vec2(half_length, half_width),
                            center: cgmath::Point2::new(center_x, center_y + half_length),
                            rotation: cgmath::Rad(0.0),
                        });
                    }
                }
                if wall_map[x][y].contains(BlockWall::BOTTOM) {
                    if y == (height - 1) as usize
                        || (y < (height - 1) as usize
                            && !wall_map[x][y + 1].contains(BlockWall::TOP))
                    {
                        horiz_boxes.push(WallBox {
                            extent: cgmath::vec2(half_length, half_width),
                            center: cgmath::point2(center_x, center_y - half_length),
                            rotation: cgmath::Rad(0.0),
                        });
                    }
                }
                if wall_map[x][y].contains(BlockWall::LEFT) {
                    if x == 0 || (x > 0 && !wall_map[x - 1][y].contains(BlockWall::RIGHT)) {
                        vert_boxes.push(WallBox {
                            extent: cgmath::vec2(half_width, half_length),
                            center: cgmath::point2(center_x - half_length, center_y),
                            rotation: cgmath::Rad(0.0),
                        });
                    }
                }
                if wall_map[x][y].contains(BlockWall::RIGHT) {
                    if x == (width - 1) as usize
                        || (x < (width - 1) as usize
                            && !wall_map[x + 1][y].contains(BlockWall::LEFT))
                    {
                        vert_boxes.push(WallBox {
                            extent: cgmath::vec2(half_width, half_length),
                            center: cgmath::point2(center_x + half_length, center_y),
                            rotation: cgmath::Rad(0.0),
                        });
                    }
                }
                let horiz_vertic = horiz_boxes
                    .iter()
                    .flat_map(|bok| {
                        [
                            Vertex {
                                pos: [bok.center.x - bok.extent.x, bok.center.y + bok.extent.y],
                                tex_coord: [1.0, 0.0],
                            },
                            Vertex {
                                pos: [bok.center.x + bok.extent.x, bok.center.y + bok.extent.y],
                                tex_coord: [1.0, 1.0],
                            },
                            Vertex {
                                pos: [bok.center.x - bok.extent.x, bok.center.y - bok.extent.y],
                                tex_coord: [0.0, 0.0],
                            },
                            Vertex {
                                pos: [bok.center.x + bok.extent.x, bok.center.y + bok.extent.y],
                                tex_coord: [1.0, 1.0],
                            },
                            Vertex {
                                pos: [bok.center.x - bok.extent.x, bok.center.y - bok.extent.y],
                                tex_coord: [0.0, 0.0],
                            },
                            Vertex {
                                pos: [bok.center.x + bok.extent.x, bok.center.y - bok.extent.y],
                                tex_coord: [0.0, 1.0],
                            },
                        ]
                    })
                    .collect::<Vec<_>>();
                let vert_vertic = vert_boxes
                    .iter()
                    .flat_map(|bok| {
                        [
                            Vertex {
                                pos: [bok.center.x - bok.extent.x, bok.center.y + bok.extent.y],
                                tex_coord: [0.0, 0.0],
                            },
                            Vertex {
                                pos: [bok.center.x + bok.extent.x, bok.center.y + bok.extent.y],
                                tex_coord: [1.0, 0.0],
                            },
                            Vertex {
                                pos: [bok.center.x - bok.extent.x, bok.center.y - bok.extent.y],
                                tex_coord: [0.0, 1.0],
                            },
                            Vertex {
                                pos: [bok.center.x + bok.extent.x, bok.center.y + bok.extent.y],
                                tex_coord: [1.0, 0.0],
                            },
                            Vertex {
                                pos: [bok.center.x - bok.extent.x, bok.center.y - bok.extent.y],
                                tex_coord: [0.0, 1.0],
                            },
                            Vertex {
                                pos: [bok.center.x + bok.extent.x, bok.center.y - bok.extent.y],
                                tex_coord: [1.0, 1.0],
                            },
                        ]
                    })
                    .collect::<Vec<_>>();
                vertices_data.extend_from_slice(&horiz_vertic);
                vertices_data.extend_from_slice(&vert_vertic);
                colliders.push(
                    horiz_boxes
                        .into_iter()
                        .map(|bok| {
                            (
                                rapier2d::na::Isometry2::translation(bok.center.x, bok.center.y),
                                ColliderBuilder::cuboid(half_length, half_width).shape,
                            )
                        })
                        .collect::<Vec<_>>(),
                );
                colliders.push(
                    vert_boxes
                        .into_iter()
                        .map(|bok| {
                            (
                                rapier2d::na::Isometry2::translation(bok.center.x, bok.center.y),
                                ColliderBuilder::cuboid(half_width, half_length).shape,
                            )
                        })
                        .collect::<Vec<_>>(),
                );
            }
        }
        let colliders = colliders.into_iter().flatten().collect::<Vec<_>>();

        let rigid_body = RigidBodyBuilder::new_static().build();
        let rigid_body_handle = rigid_body_set.insert(rigid_body);
        collider_set.insert_with_parent(
            ColliderBuilder::compound(colliders).friction(0.0).build(),
            rigid_body_handle,
            rigid_body_set,
        );

        let vertices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Wall Vertices"),
            contents: bytemuck::cast_slice(&vertices_data),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let identity_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Wall Transform"),
            contents: bytemuck::cast_slice(&[Data {
                color: [1.0, 1.0, 1.0],
                _padding: 0,
                transform: [
                    [1.0, 0.0, 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
            }]),
            usage: wgpu::BufferUsages::UNIFORM,
        });
        let transform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Wall Transform"),
            layout: transform_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: identity_buffer.as_entire_binding(),
            }],
        });

        let raw_wall_texture = include_bytes!("wall.png");
        let wall_texture_data = image::load_from_memory(raw_wall_texture).unwrap();
        let wall_rgba = wall_texture_data.as_rgba8().unwrap();
        let (wall_x, wall_y) = wall_texture_data.dimensions();

        let wall_texture = device.create_texture_with_data(
            queue,
            &wgpu::TextureDescriptor {
                label: Some("Wall Texture"),
                size: wgpu::Extent3d {
                    width: wall_x,
                    height: wall_y,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba8UnormSrgb,
                usage: wgpu::TextureUsages::TEXTURE_BINDING,
            },
            &wall_rgba,
        );

        let wall_texture_view = wall_texture.create_view(&wgpu::TextureViewDescriptor {
            label: Some("Wall Texture View"),
            format: Some(wgpu::TextureFormat::Rgba8UnormSrgb),
            dimension: Some(wgpu::TextureViewDimension::D2),
            ..Default::default()
        });

        let asset_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Asset Bind"),
            layout: asset_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&wall_texture_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(sampler),
                },
            ],
        });
        Self {
            vertices,
            vertices_count: vertices_data.len() as _,
            transform_bind_group,
            asset_bind_group,
        }
    }

    pub fn draw<'a>(&'a self, render_pass: &mut wgpu::RenderPass<'a>) {
        render_pass.set_bind_group(0, &self.transform_bind_group, &[]);
        render_pass.set_bind_group(2, &self.asset_bind_group, &[]);
        render_pass.set_vertex_buffer(0, self.vertices.slice(..));
        render_pass.draw(0..self.vertices_count, 0..1);
    }
}
