// #![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]
use std::{
    collections::HashMap,
    time::{Duration, Instant},
};

use arena::Arena;
use argh::FromArgs;
use bytemuck::{Pod, Zeroable};
use cannon::CannonBallManager;
use log::{debug, info, LevelFilter};
use message_io::{
    network::{Endpoint, Transport},
    node::{NodeHandler, NodeListener},
};
use rapier2d::prelude::*;
use tank_conflict_server::protocol::{ClientDataPacket, DataPacket};
use uuid::Uuid;
use wgpu::util::DeviceExt;
use winit::event_loop::ControlFlow;

mod arena;
mod cannon;

#[rustfmt::skip]
pub const OPENGL_TO_WGPU_MATRIX: cgmath::Matrix4<f32> = cgmath::Matrix4::new(
    1.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 0.0, 0.0,
    0.0, 0.0, 0.5, 0.0,
    0.0, 0.0, 0.5, 1.0,
);

#[repr(C)]
#[derive(Debug, Clone, Copy, Zeroable, Pod)]
pub struct Vertex {
    pos: [f32; 2],
    tex_coord: [f32; 2],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Zeroable, Pod)]
struct Data {
    color: [f32; 3],
    _padding: u32,
    transform: [[f32; 4]; 4],
}

#[repr(C)]
#[derive(Debug, Clone, Copy, Zeroable, Pod)]
struct WindowData {
    transform: [[f32; 4]; 4],
}

struct Tank {
    position: cgmath::Vector2<f32>,
    rotation: cgmath::Rad<f32>,

    uuid: uuid::Uuid,

    physics_body_handle: RigidBodyHandle,

    bind_group: wgpu::BindGroup,
    buffer: wgpu::Buffer,
    color: [f32; 3],

    dirty: bool,
}

impl Tank {
    pub fn new<P: Into<cgmath::Vector2<f32>>, R: Into<cgmath::Rad<f32>>>(
        pos: P,
        rot: R,
        color: [f32; 3],
        uuid: uuid::Uuid,
        device: &wgpu::Device,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
    ) -> Self {
        let position = pos.into();
        let rotation = rot.into();

        let trans =
            cgmath::Matrix4::from_translation(cgmath::Vector3::new(position.x, position.y, 0.0));
        let rot = cgmath::Matrix4::from_angle_z(rotation);
        let transform = trans * rot * cgmath::Matrix4::from_scale(0.7);

        let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Tank Data"),
            contents: bytemuck::cast_slice(&[Data {
                color,
                _padding: 0,
                transform: transform.into(),
            }]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let bind_group = {
            let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("Tank Data Bind"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });
            device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("Tank Bind"),
                layout: &layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: buffer.as_entire_binding(),
                }],
            })
        };

        let collider_body = ColliderBuilder::cuboid(36.5 * 0.7, 43.0 * 0.7).shape;
        let collider_cannon = ColliderBuilder::cuboid(4.5 * 0.7, 17.5 * 0.7).shape;

        let collider = ColliderBuilder::compound(vec![
            (
                rapier2d::na::Isometry2::translation(0.0, -13.5 * 0.7),
                collider_body,
            ),
            (
                rapier2d::na::Isometry2::translation(0.0, 39.0 * 0.7),
                collider_cannon,
            ),
        ])
        .active_events(ActiveEvents::CONTACT_EVENTS)
        .build();

        let rigid_body = RigidBodyBuilder::new_dynamic()
            .translation(vector![position.x, position.y])
            .rotation(rotation.0)
            .build();

        let physics_body_handle = rigid_body_set.insert(rigid_body);
        collider_set.insert_with_parent(collider, physics_body_handle, rigid_body_set);

        Self {
            position,
            rotation,
            bind_group,
            buffer,
            color,
            uuid,
            dirty: false,
            physics_body_handle,
        }
    }

    pub fn bind<'a>(&'a mut self, queue: &wgpu::Queue, render_pass: &mut wgpu::RenderPass<'a>) {
        if self.dirty {
            self.update_data(queue);
            self.dirty = false;
        }
        render_pass.set_bind_group(0, &self.bind_group, &[]);
    }

    fn update_data(&self, queue: &wgpu::Queue) {
        let trans = cgmath::Matrix4::from_translation(cgmath::Vector3::new(
            self.position.x,
            self.position.y,
            0.0,
        ));
        let rot = cgmath::Matrix4::from_angle_z(self.rotation);
        let transform = trans * rot * cgmath::Matrix4::from_scale(0.7);
        queue.write_buffer(
            &self.buffer,
            0,
            bytemuck::cast_slice(&[Data {
                color: self.color,
                _padding: 0,
                transform: transform.into(),
            }]),
        )
    }

    fn sync_tank_pose(&mut self, rigid_body_set: &RigidBodySet) {
        let rotation = rigid_body_set[self.physics_body_handle].rotation().angle();
        let position = rigid_body_set[self.physics_body_handle].translation();
        self.rotation = cgmath::Rad(rotation);
        self.position = cgmath::vec2(position.x, position.y);
        self.dirty = true;
    }
}

struct PhysicsData {
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    integration_params: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    joint_set: JointSet,
    ccd_solver: CCDSolver,
}

impl PhysicsData {
    fn new() -> Self {
        Self {
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            integration_params: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            joint_set: JointSet::new(),
            ccd_solver: CCDSolver::new(),
        }
    }
}

#[derive(Default, Debug)]
struct ControlState {
    w_pressed: bool,
    s_pressed: bool,
    a_pressed: bool,
    d_pressed: bool,
    space_pressed: bool,
}

struct Player {
    uuid: Uuid,
    color: [f32; 3],
    score: u32,
    name: String,
}

struct Application {
    surface: wgpu::Surface,
    device: wgpu::Device,
    queue: wgpu::Queue,
    surface_configuration: wgpu::SurfaceConfiguration,
    pipeline: wgpu::RenderPipeline,

    tanks: HashMap<Uuid, Tank>,
    scoreboard: HashMap<Uuid, Player>,
    client_collider_handle: ColliderHandle,
    client_uuid: Uuid,
    cannon_cooldown: u32,

    arena: Option<Arena>,
    cannon_balls: CannonBallManager,

    data_layout: wgpu::BindGroupLayout,

    projection: cgmath::Matrix4<f32>,
    bind_group: wgpu::BindGroup,
    buffer: wgpu::Buffer,
    tank_vertices: wgpu::Buffer,
    tank_texture_group: wgpu::BindGroup,

    physics: PhysicsData,
    control: ControlState,

    listener: NodeListener<()>,
    handler: NodeHandler<()>,
    server: Endpoint,
    option: GameOptions,
    receiver: crossbeam::channel::Receiver<()>,

    tau: Duration,
    now: Instant,
    lag: Duration,
}

impl Application {
    pub fn new(
        width: u32,
        height: u32,
        title: &str,
        tps: u32,
        opts: GameOptions,
    ) -> anyhow::Result<(
        Self,
        winit::event_loop::EventLoop<()>,
        winit::window::Window,
    )> {
        let event_loop = winit::event_loop::EventLoop::new();
        let window = winit::window::WindowBuilder::new()
            .with_inner_size(winit::dpi::PhysicalSize::new(width, height))
            .with_title(title)
            .with_resizable(false)
            .build(&event_loop)?;
        let instance = wgpu::Instance::new(wgpu::Backends::all());
        let surface = unsafe { instance.create_surface(&window) };
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            force_fallback_adapter: false,
            compatible_surface: Some(&surface),
        }))
        .unwrap();
        let (device, queue) = pollster::block_on(adapter.request_device(
            &wgpu::DeviceDescriptor {
                label: Some("Main Device"),
                features: wgpu::Features::empty(),
                limits: wgpu::Limits {
                    ..wgpu::Limits::downlevel_defaults()
                },
            },
            None,
        ))?;

        let surface_configuration = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format: surface.get_preferred_format(&adapter).unwrap(),
            width,
            height,
            present_mode: wgpu::PresentMode::Mailbox,
        };

        surface.configure(&device, &surface_configuration);

        let data_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Data Bind"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let window_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Data Bind"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let asset_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("Tank Textures"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler {
                        filtering: true,
                        comparison: false,
                    },
                    count: None,
                },
            ],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("Blit Layout"),
            bind_group_layouts: &[&data_layout, &window_layout, &asset_layout],
            push_constant_ranges: &[],
        });
        let vertex_module = device.create_shader_module(&wgpu::include_wgsl!("blit.wgsl"));
        let fragment_module = device.create_shader_module(&wgpu::include_wgsl!("blit.wgsl"));

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("Blit Pipe"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &vertex_module,
                entry_point: "main",
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as _,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![
                        0 => Float32x2,
                        1 => Float32x2,
                    ],
                }],
            },
            primitive: wgpu::PrimitiveState::default(),
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            fragment: Some(wgpu::FragmentState {
                module: &fragment_module,
                entry_point: "main",
                targets: &[wgpu::ColorTargetState {
                    format: surface_configuration.format,
                    blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                }],
            }),
        });

        let projection = OPENGL_TO_WGPU_MATRIX
            * cgmath::ortho(
                -0.5 * width as f32,
                0.5 * width as f32,
                -0.5 * height as f32,
                0.5 * height as f32,
                0.0,
                1.0,
            );

        let proj_data = WindowData {
            transform: projection.into(),
        };
        let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Window Buffer"),
            contents: bytemuck::cast_slice(&[proj_data]),
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        });

        let bind_group = {
            device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("Window Bind"),
                layout: &window_layout,
                entries: &[wgpu::BindGroupEntry {
                    binding: 0,
                    resource: buffer.as_entire_binding(),
                }],
            })
        };

        let tank_picture = include_bytes!("tank.png");
        let tank_picture = image::load_from_memory(tank_picture)?;
        let tank_rgba = tank_picture.as_rgba8().unwrap();
        let (tank_width, tank_height) = tank_rgba.dimensions();

        let tank_texture = device.create_texture_with_data(
            &queue,
            &wgpu::TextureDescriptor {
                label: Some("Tank Layout Texture"),
                size: wgpu::Extent3d {
                    width: tank_width,
                    height: tank_height,
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba8UnormSrgb,
                usage: wgpu::TextureUsages::TEXTURE_BINDING,
            },
            tank_rgba,
        );

        let tank_texture_view = tank_texture.create_view(&wgpu::TextureViewDescriptor {
            label: Some("Tank Texture View"),
            format: Some(wgpu::TextureFormat::Rgba8UnormSrgb),
            dimension: Some(wgpu::TextureViewDimension::D2),
            ..Default::default()
        });

        let default_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("Default Sampler"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });

        let tank_texture_group = {
            device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("Tank Texture Group"),
                layout: &asset_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::TextureView(&tank_texture_view),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::Sampler(&default_sampler),
                    },
                ],
            })
        };

        let vertices = {
            let half_width = tank_width as f32 * 0.5;
            let half_height = tank_height as f32 * 0.5;
            [
                Vertex {
                    pos: [-half_width, half_height],
                    tex_coord: [0.0, 0.0],
                },
                Vertex {
                    pos: [half_width, half_height],
                    tex_coord: [1.0, 0.0],
                },
                Vertex {
                    pos: [-half_width, -half_height],
                    tex_coord: [0.0, 1.0],
                },
                Vertex {
                    pos: [half_width, half_height],
                    tex_coord: [1.0, 0.0],
                },
                Vertex {
                    pos: [half_width, -half_height],
                    tex_coord: [1.0, 1.0],
                },
                Vertex {
                    pos: [-half_width, -half_height],
                    tex_coord: [0.0, 1.0],
                },
            ]
        };
        let tank_vertices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("Tank Vertices"),
            contents: bytemuck::cast_slice(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        let mut physics = PhysicsData::new();

        let cannon_balls = CannonBallManager::new(&device, &queue, &asset_layout, &default_sampler);

        let (handler, listener) = message_io::node::split();

        let (server, _) = handler
            .network()
            .connect(Transport::FramedTcp, (opts.server, opts.port))
            .unwrap();

        Ok((
            Self {
                surface,
                device,
                queue,
                surface_configuration,
                pipeline,
                tanks: HashMap::new(),
                client_collider_handle: ColliderHandle::invalid(),
                arena: None,
                projection,
                bind_group,
                buffer,
                tank_vertices,
                tank_texture_group,
                now: Instant::now(),
                lag: Duration::ZERO,
                tau: Duration::from_secs_f64(1.0 / tps as f64),
                physics,
                control: Default::default(),
                cannon_balls,
                data_layout,
                cannon_cooldown: 0,
                client_uuid: Uuid::new_v4(),
                listener,
                handler,
                scoreboard: HashMap::new(),
                server,
                option: opts,
            },
            event_loop,
            window,
        ))
    }

    pub fn resize(&mut self, new_size: winit::dpi::PhysicalSize<u32>) {
        self.surface_configuration.width = new_size.width;
        self.surface_configuration.height = new_size.height;
        self.surface
            .configure(&self.device, &self.surface_configuration);
        self.projection = OPENGL_TO_WGPU_MATRIX
            * cgmath::ortho(
                new_size.width as f32 * -0.5,
                new_size.width as f32 * 0.5,
                new_size.height as f32 * -0.5,
                new_size.height as f32 * 0.5,
                0.0,
                1.0,
            );
        self.queue.write_buffer(
            &self.buffer,
            0,
            bytemuck::cast_slice(&[WindowData {
                transform: self.projection.into(),
            }]),
        );
    }

    pub fn spawn_tank<P: Into<cgmath::Vector2<f32>>, R: Into<cgmath::Rad<f32>>>(
        &mut self,
        pos: P,
        rot: R,
        color: [f32; 3],
        uuid: uuid::Uuid,
    ) {
        let tank = Tank::new(
            pos,
            rot,
            color,
            uuid,
            &self.device,
            &mut self.physics.rigid_body_set,
            &mut self.physics.collider_set,
        );
        self.tanks.insert(uuid, tank);
    }

    pub fn input(&mut self, input: winit::event::KeyboardInput) {
        if let winit::event::KeyboardInput {
            virtual_keycode: Some(code),
            state,
            ..
        } = input
        {
            match code {
                winit::event::VirtualKeyCode::W => {
                    self.control.w_pressed = state == winit::event::ElementState::Pressed;
                }
                winit::event::VirtualKeyCode::A => {
                    self.control.a_pressed = state == winit::event::ElementState::Pressed;
                }
                winit::event::VirtualKeyCode::S => {
                    self.control.s_pressed = state == winit::event::ElementState::Pressed;
                }
                winit::event::VirtualKeyCode::D => {
                    self.control.d_pressed = state == winit::event::ElementState::Pressed;
                }
                winit::event::VirtualKeyCode::Space => {
                    self.control.space_pressed = state == winit::event::ElementState::Pressed;
                }
                _ => (),
            }
        }
    }

    pub fn update(&mut self, dt: Duration) {
        let _dt = dt.as_secs_f32();
        if self.control.w_pressed && !self.control.s_pressed {
            let (dir_x, dir_y) = self.physics.rigid_body_set
                [self.tanks[&self.client_uuid].physics_body_handle]
                .rotation()
                .angle()
                .sin_cos();
            self.physics.rigid_body_set[self.tanks[&self.client_uuid].physics_body_handle]
                .set_linvel(vector![-dir_x * 80.0, dir_y * 80.0], true);
        }
        if self.control.s_pressed && !self.control.w_pressed {
            let (dir_x, dir_y) = self.physics.rigid_body_set
                [self.tanks[&self.client_uuid].physics_body_handle]
                .rotation()
                .angle()
                .sin_cos();
            self.physics.rigid_body_set[self.tanks[&self.client_uuid].physics_body_handle]
                .set_linvel(-vector![-dir_x * 80.0, dir_y * 80.0], true);
        }
        if !(self.control.w_pressed ^ self.control.s_pressed) {
            self.physics.rigid_body_set[self.tanks[&self.client_uuid].physics_body_handle]
                .set_linvel(vector![0.0, 0.0], true);
        }
        if self.control.a_pressed && !self.control.d_pressed {
            self.physics.rigid_body_set[self.tanks[&self.client_uuid].physics_body_handle]
                .set_angvel(std::f32::consts::FRAC_PI_3, true)
        }
        if self.control.d_pressed && !self.control.a_pressed {
            self.physics.rigid_body_set[self.tanks[&self.client_uuid].physics_body_handle]
                .set_angvel(-std::f32::consts::FRAC_PI_3, true)
        }
        if !(self.control.a_pressed ^ self.control.d_pressed) {
            self.physics.rigid_body_set[self.tanks[&self.client_uuid].physics_body_handle]
                .set_angvel(0.0, true)
        }
        if self.control.space_pressed && self.cannon_cooldown == 0 {
            let (ori_sin, ori_cos) = self.physics.rigid_body_set
                [self.tanks[&self.client_uuid].physics_body_handle]
                .rotation()
                .angle()
                .sin_cos();
            let tank_pos = self.physics.rigid_body_set
                [self.tanks[&self.client_uuid].physics_body_handle]
                .translation();
            if self.cannon_balls.generate_new_cannon_ball(
                [tank_pos.x - 45.0 * ori_sin, tank_pos.y + 45.0 * ori_cos],
                [90.0 * -ori_sin, 90.0 * ori_cos],
                600,
                cannon::CannonOwner::Me,
                &self.data_layout,
                &self.device,
                &mut self.physics.rigid_body_set,
                &mut self.physics.collider_set,
            ) {
                let data_packet = DataPacket::C2S(
                    self.client_uuid,
                    ClientDataPacket::BulletsFired {
                        position: [tank_pos.x - 45.0 * ori_sin, tank_pos.y + 45.0 * ori_cos],
                        velocity: [90.0 * -ori_sin, 90.0 * ori_cos],
                    },
                );
                self.handler
                    .network()
                    .send(self.server, &serde_json::to_vec(&data_packet).unwrap());
                self.cannon_cooldown = 15;
            }
        } else if self.cannon_cooldown > 0 {
            self.cannon_cooldown -= 1;
        }
        self.cannon_balls.tick(
            &self.queue,
            &mut self.physics.rigid_body_set,
            &mut self.physics.collider_set,
            &mut self.physics.island_manager,
            &mut self.physics.joint_set,
        );
        let physics_hooks = ();
        let (intersec_send, _) = crossbeam::channel::unbounded();
        let (contact_send, contact_recv) = crossbeam::channel::unbounded();
        let event_handler = ChannelEventCollector::new(intersec_send, contact_send);
        self.physics.physics_pipeline.step(
            &vector![0.0, 0.0],
            &self.physics.integration_params,
            &mut self.physics.island_manager,
            &mut self.physics.broad_phase,
            &mut self.physics.narrow_phase,
            &mut self.physics.rigid_body_set,
            &mut self.physics.collider_set,
            &mut self.physics.joint_set,
            &mut self.physics.ccd_solver,
            &physics_hooks,
            &event_handler,
        );
        let current_position = self.physics.rigid_body_set
            [self.tanks[&self.client_uuid].physics_body_handle]
            .translation();
        let current_angle = self.physics.rigid_body_set
            [self.tanks[&self.client_uuid].physics_body_handle]
            .rotation()
            .angle();
        let data_packet = DataPacket::C2S(
            self.client_uuid,
            ClientDataPacket::TankPose {
                position: [current_position.x, current_position.y],
                angle: current_angle,
            },
        );
        self.handler
            .network()
            .send(self.server, &serde_json::to_vec(&data_packet).unwrap());
        while let Ok(contact_event) = contact_recv.try_recv() {
            if let ContactEvent::Started(ha, hb) = contact_event {
                self.cannon_balls.remove_hit(
                    ha,
                    &mut self.physics.rigid_body_set,
                    &mut self.physics.collider_set,
                    &mut self.physics.island_manager,
                    &mut self.physics.joint_set,
                );
                self.cannon_balls.remove_hit(
                    hb,
                    &mut self.physics.rigid_body_set,
                    &mut self.physics.collider_set,
                    &mut self.physics.island_manager,
                    &mut self.physics.joint_set,
                );
                if ha == self.client_collider_handle || hb == self.client_collider_handle {
                    let data_packet =
                        DataPacket::C2S(self.client_uuid, ClientDataPacket::Destroyed);
                    self.handler
                        .network()
                        .send(self.server, &serde_json::to_vec(&data_packet).unwrap());
                    todo!("death logic")
                }
            }
        }
        self.tanks
            .iter_mut()
            .for_each(|(_, tank)| tank.sync_tank_pose(&self.physics.rigid_body_set));
    }

    pub fn render(&mut self, _dt: Duration) {
        let surface_texture = match self.surface.get_current_texture() {
            Ok(texture) => texture,
            Err(err) => match err {
                wgpu::SurfaceError::Timeout => return,
                wgpu::SurfaceError::Outdated => return,
                wgpu::SurfaceError::Lost => return,
                wgpu::SurfaceError::OutOfMemory => panic!("MEMORY OUT RUN"),
            },
        };
        let view = surface_texture
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("Main Encoder"),
            });

        {
            let mut render_pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("Tank Pass"),
                color_attachments: &[wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::WHITE),
                        store: true,
                    },
                }],
                depth_stencil_attachment: None,
            });
            render_pass.set_pipeline(&self.pipeline);
            render_pass.set_vertex_buffer(0, self.tank_vertices.slice(..));
            render_pass.set_bind_group(1, &self.bind_group, &[]);
            render_pass.set_bind_group(2, &self.tank_texture_group, &[]);
            self.tanks.iter_mut().for_each(|(_, tank)| {
                tank.bind(&self.queue, &mut render_pass);
                render_pass.draw(0..6, 0..1);
            });
            self.cannon_balls.draw(&mut render_pass);
            if let Some(ref arena) = self.arena {
                self.arena.draw(&mut render_pass);
            }
        }

        self.queue.submit(std::iter::once(encoder.finish()));

        surface_texture.present();
    }
}

/// Option before you start the game
#[derive(FromArgs)]
struct GameOptions {
    /// your name displayed
    #[argh(option)]
    nickname: String,
    /// color of your tank, 0-255
    #[argh(option)]
    color: [u8; 3],
    /// server address
    #[argh(option)]
    server: String,
    /// server port
    #[argh(option, default = "14532")]
    port: u16,
}

fn main() -> anyhow::Result<()> {
    env_logger::builder()
        .filter_level(LevelFilter::Warn)
        .filter_module("tank_conflict", LevelFilter::Debug)
        .init();

    let opts: GameOptions = argh::from_env();

    let (mut app, event_loop, window) = Application::new(800, 600, "Tank Conflict", 60, opts)?;

    event_loop.run(move |event, _, control_flow| match event {
        winit::event::Event::WindowEvent { window_id, event } if window_id == window.id() => {
            match event {
                winit::event::WindowEvent::CloseRequested => *control_flow = ControlFlow::Exit,
                winit::event::WindowEvent::Resized(size) => app.resize(size),
                winit::event::WindowEvent::ScaleFactorChanged { new_inner_size, .. } => {
                    app.resize(*new_inner_size)
                }
                winit::event::WindowEvent::KeyboardInput { input, .. } => app.input(input),
                _ => (),
            }
        }
        winit::event::Event::MainEventsCleared => window.request_redraw(),
        winit::event::Event::RedrawRequested(_) => {
            let dt = app.now.elapsed();
            app.now = Instant::now();
            app.lag += dt;
            while app.lag > app.tau {
                app.update(app.tau);
                app.lag -= app.tau;
            }
            app.render(dt);
        }
        _ => (),
    });
}
