use std::collections::HashMap;

use image::GenericImageView;
use rapier2d::prelude::*;
use retain_mut::RetainMut;
use wgpu::util::DeviceExt;

use crate::{Data, Vertex};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CannonOwner {
    Me,
    Others,
}

pub struct CannonBall {
    data_buffer: wgpu::Buffer,
    data_bind_group: wgpu::BindGroup,
    lifetime: u32,
    rigid_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    owner: CannonOwner,
}

impl CannonBall {
    pub fn new<P: Into<cgmath::Point2<f32>>, V: Into<cgmath::Vector2<f32>>>(
        position: P,
        velocity: V,
        lifetime: u32,
        owner: CannonOwner,
        data_layout: &wgpu::BindGroupLayout,
        device: &wgpu::Device,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
    ) -> Self {
        let position = position.into();
        let velocity: cgmath::Vector2<f32> = velocity.into();

        let translation =
            cgmath::Matrix4::from_translation(cgmath::vec3(position.x, position.y, 0.0));

        let data_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Cannon Ball Data"),
            contents: bytemuck::cast_slice(&[Data {
                color: [0.0, 0.0, 0.0],
                _padding: 0,
                transform: translation.into(),
            }]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let data_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Cannon Ball Data Bind"),
            layout: data_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: data_buffer.as_entire_binding(),
            }],
        });

        let collider = ColliderBuilder::ball(4.5)
            .friction(0.0)
            .restitution_combine_rule(CoefficientCombineRule::Max)
            .restitution(1.0)
            .build();
        let rigid_body = RigidBodyBuilder::new_dynamic()
            .linvel(vector![velocity.x, velocity.y])
            .translation(vector![position.x, position.y])
            .build();
        let rb_handle = rigid_body_set.insert(rigid_body);
        let collider_handle = collider_set.insert_with_parent(collider, rb_handle, rigid_body_set);

        Self {
            data_buffer,
            data_bind_group,
            lifetime,
            rigid_handle: rb_handle,
            owner,
            collider_handle,
        }
    }

    pub fn tick(
        &mut self,
        queue: &wgpu::Queue,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        island_manager: &mut IslandManager,
        joint_set: &mut JointSet,
    ) -> bool {
        self.lifetime -= 1;
        if self.lifetime == 0 {
            rigid_body_set.remove(self.rigid_handle, island_manager, collider_set, joint_set);
            return true;
        }
        let new_pos = rigid_body_set[self.rigid_handle].translation();
        let translation =
            cgmath::Matrix4::from_translation(cgmath::vec3(new_pos.x, new_pos.y, 0.0));

        queue.write_buffer(
            &self.data_buffer,
            0,
            bytemuck::cast_slice(&[Data {
                color: [0.0, 0.0, 0.0],
                _padding: 0,
                transform: translation.into(),
            }]),
        );

        return false;
    }
}

pub struct CannonBallManager {
    asset_bind_group: wgpu::BindGroup,
    vertices: wgpu::Buffer,
    bullets: Vec<CannonBall>,
    client_bullets: HashMap<ColliderHandle, RigidBodyHandle>,
}

impl CannonBallManager {
    pub fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        asset_layout: &wgpu::BindGroupLayout,
        sampler: &wgpu::Sampler,
    ) -> Self {
        let cannon_data = image::load_from_memory(include_bytes!("cannon_ball.png")).unwrap();
        let cannon_data_rgba = cannon_data.as_rgba8().unwrap();
        let dimension = cannon_data.dimensions();
        let texture = device.create_texture_with_data(
            queue,
            &wgpu::TextureDescriptor {
                label: Some("Cannon Ball Texture"),
                size: wgpu::Extent3d {
                    width: dimension.0,
                    height: dimension.1,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba8UnormSrgb,
                usage: wgpu::TextureUsages::TEXTURE_BINDING,
            },
            cannon_data_rgba,
        );
        let texture = texture.create_view(&wgpu::TextureViewDescriptor {
            label: Some("Cannon Ball Texture View"),
            format: Some(wgpu::TextureFormat::Rgba8UnormSrgb),
            dimension: Some(wgpu::TextureViewDimension::D2),
            ..Default::default()
        });
        let asset_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("Cannon Ball Asset Bind"),
            layout: asset_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&texture),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(sampler),
                },
            ],
        });

        let vertices = [
            Vertex {
                pos: [-4.5, 4.5],
                tex_coord: [0.0, 0.0],
            },
            Vertex {
                pos: [4.5, 4.5],
                tex_coord: [1.0, 0.0],
            },
            Vertex {
                pos: [-4.5, -4.5],
                tex_coord: [0.0, 1.0],
            },
            Vertex {
                pos: [4.5, 4.5],
                tex_coord: [1.0, 0.0],
            },
            Vertex {
                pos: [-4.5, -4.5],
                tex_coord: [0.0, 1.0],
            },
            Vertex {
                pos: [4.5, -4.5],
                tex_coord: [1.0, 1.0],
            },
        ];

        let vertices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Cannon Ball Vertex"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        Self {
            asset_bind_group,
            vertices,
            bullets: vec![],
            client_bullets: HashMap::new(),
        }
    }

    pub fn generate_new_cannon_ball<P: Into<cgmath::Point2<f32>>, V: Into<cgmath::Vector2<f32>>>(
        &mut self,
        position: P,
        velocity: V,
        lifetime: u32,
        owner: CannonOwner,
        data_layout: &wgpu::BindGroupLayout,
        device: &wgpu::Device,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
    ) -> bool {
        let new_ball = if owner == CannonOwner::Me && self.client_bullets.len() <= 4 {
            let new_ball = CannonBall::new(
                position,
                velocity,
                lifetime,
                owner,
                data_layout,
                device,
                rigid_body_set,
                collider_set,
            );
            self.client_bullets
                .insert(new_ball.collider_handle, new_ball.rigid_handle);
            new_ball
        } else if owner == CannonOwner::Me && self.client_bullets.len() > 4 {
            return false;
        } else {
            CannonBall::new(
                position,
                velocity,
                lifetime,
                owner,
                data_layout,
                device,
                rigid_body_set,
                collider_set,
            )
        };
        self.bullets.push(new_ball);
        return true;
    }

    pub fn remove_hit(
        &mut self,
        collider: ColliderHandle,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        island_manager: &mut IslandManager,
        joint_set: &mut JointSet,
    ) {
        if self.client_bullets.contains_key(&collider) {
            rigid_body_set.remove(
                self.client_bullets[&collider],
                island_manager,
                collider_set,
                joint_set,
            );
            self.client_bullets.remove(&collider);
            self.bullets
                .retain(|cannon| !(cannon.collider_handle == collider))
        }
    }

    pub fn tick(
        &mut self,
        queue: &wgpu::Queue,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        island_manager: &mut IslandManager,
        joint_set: &mut JointSet,
    ) {
        self.bullets.retain_mut(|bullet| {
            let aged = bullet.tick(
                queue,
                rigid_body_set,
                collider_set,
                island_manager,
                joint_set,
            );
            if aged {
                self.client_bullets.remove(&bullet.collider_handle);
            }
            !aged
        });
    }

    pub fn draw<'a>(&'a self, render_pass: &mut wgpu::RenderPass<'a>) {
        render_pass.set_bind_group(2, &self.asset_bind_group, &[]);
        render_pass.set_vertex_buffer(0, self.vertices.slice(..));
        self.bullets.iter().for_each(|cannon| {
            render_pass.set_bind_group(0, &cannon.data_bind_group, &[]);
            render_pass.draw(0..6, 0..1);
        });
    }
}
