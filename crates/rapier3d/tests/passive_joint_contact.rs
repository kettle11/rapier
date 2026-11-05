//! Passive high-mass-ratio collision regression. Rebuilt with public APIs and
//! no cached impulses, motors, restitution, tire hooks, or application code.
#![cfg(feature = "serde-serialize")]
use rapier::prelude::*;
use rapier3d as rapier;
use serde_json::{Value, json};
fn energy(w: &PhysicsWorld) -> f64 {
    w.bodies
        .iter()
        .map(|(_, b)| {
            b.kinetic_energy() as f64 + b.mass() as f64 * 9.81 * b.center_of_mass().y as f64
        })
        .sum()
}
fn run_scene(mode: &str, scene: &str, steps: usize) -> Value {
    let s: Value = serde_json::from_str(scene).unwrap();
    let mut w = PhysicsWorld::new();
    w.integration_parameters.dt = 1.0 / 120.0;
    w.integration_parameters.num_internal_pgs_iterations = 4;
    let mut hs = vec![];
    for name in ["chassis", "wheel"] {
        let q = &s[name];
        let pose: Pose = serde_json::from_value(q["pose"].clone()).unwrap();
        let v: Vector = serde_json::from_value(q["linvel"].clone()).unwrap();
        let av: Vector = serde_json::from_value(q["angvel"].clone()).unwrap();
        let m: MassProperties = serde_json::from_value(q["mass"].clone()).unwrap();
        hs.push(
            w.bodies.insert(
                RigidBodyBuilder::dynamic()
                    .pose(pose)
                    .linvel(v)
                    .angvel(if mode == "nospin" { Vector::ZERO } else { av })
                    .additional_mass_properties(m)
                    .can_sleep(false)
                    .ccd_enabled(true)
                    .additional_solver_iterations(q["extra_iterations"].as_u64().unwrap() as usize),
            ),
        );
    }
    let pts: Vec<Vector> =
        serde_json::from_str(include_str!("fixtures/passive-contact/wheel-points.json")).unwrap();
    let shape = SharedShape::round_convex_hull(&pts, 0.03).unwrap();
    if mode == "coulomb" {
        w.integration_parameters.friction_model = FrictionModel::Coulomb;
    }
    w.colliders.insert_with_parent(
        ColliderBuilder::new(shape)
            .position(serde_json::from_value::<Pose>(s["collider_pose"].clone()).unwrap())
            .density(0.0)
            .friction(if mode == "frictionless" { 0.0 } else { 1.2 })
            .restitution(0.0)
            .build(),
        hs[1],
        &mut w.bodies,
    );
    let normal = Vector::new(-0.18783034, 0.95508057, -0.22921799).normalize();
    let point = Vector::new(-15.631003, 139.45126, 409.93903);
    w.colliders.insert(
        ColliderBuilder::halfspace(rapier::na::Unit::new_unchecked(normal))
            .translation(point)
            .friction(if mode == "frictionless" { 0.0 } else { 0.9 })
            .restitution(0.0),
    );
    // Ordinary passive slider/axle joint; no motors, cached impulses, or damage state.
    let joint = GenericJointBuilder::new(
        JointAxesMask::LIN_X | JointAxesMask::LIN_Z | JointAxesMask::ANG_Y | JointAxesMask::ANG_Z,
    )
    .local_frame1(serde_json::from_value::<Pose>(s["joint"]["local_frame1"].clone()).unwrap())
    .local_frame2(serde_json::from_value::<Pose>(s["joint"]["local_frame2"].clone()).unwrap())
    .limits(JointAxis::LinY, [-0.45, 0.45])
    .contacts_enabled(false)
    .build();
    w.insert_impulse_joint(hs[0], hs[1], joint);
    for (_, b) in w.bodies.iter_mut() {
        b.recompute_mass_properties_from_colliders(&w.colliders);
    }
    let before = energy(&w);
    let ke_before: f32 = w.bodies.iter().map(|(_, b)| b.kinetic_energy()).sum();
    let mut peak_gain = 0.0_f64;
    let mut had_contact = false;
    for _ in 0..steps {
        w.step();
        let current = energy(&w);
        assert!(current.is_finite(), "non-finite state in {mode}");
        peak_gain = peak_gain.max(current - before);
        had_contact |= w
            .narrow_phase
            .contact_pairs()
            .any(|pair| pair.has_any_active_contact());
    }
    assert!(had_contact, "the {mode} fixture must actually collide");
    let after = energy(&w);
    json!({"mode":mode,"peak_energy_gain_J":peak_gain,"vertical_velocity":w.bodies[hs[0]].linvel().y,"energy_gain_J":after-before,"kinetic_before_J":ke_before,"kinetic_after_J":w.bodies.iter().map(|(_,b)|b.kinetic_energy()).sum::<f32>()})
}

fn check(data: &str, steps: usize, mode: &str) {
    let result = run_scene(mode, data, steps);
    let gain = result["energy_gain_J"].as_f64().unwrap();
    eprintln!("{result}");
    // One joule allows floating-point noise relative to ~100 kJ incident KE.
    // Before the fix: +155 kJ (impact), +23 kJ (approach), +171 kJ (no spin).
    assert!(gain <= 1.0, "passive collision created energy: {result}");
}

#[test]
fn passive_wheel_impact_does_not_launch_jointed_chassis() {
    check(
        include_str!("fixtures/passive-contact/scene.json"),
        1,
        "baseline",
    );
}

#[test]
fn well_aligned_approach_does_not_generate_energy() {
    // Starts with <0.05 mm locked-axis error, before the contact: the simulation
    // itself creates the strained joint. This is not merely bad initial state.
    check(
        include_str!("fixtures/passive-contact/approach-scene.json"),
        3,
        "baseline",
    );
}

#[test]
fn passive_impact_does_not_require_spin_or_friction() {
    for mode in ["nospin", "frictionless"] {
        check(include_str!("fixtures/passive-contact/scene.json"), 1, mode);
    }
}

#[test]
fn passive_impact_with_coulomb_friction() {
    check(
        include_str!("fixtures/passive-contact/scene.json"),
        1,
        "coulomb",
    );
}

#[test]
fn passive_impact_does_not_defer_the_launch_to_later_steps() {
    for mode in ["baseline", "coulomb", "nospin", "frictionless"] {
        let result = run_scene(
            mode,
            include_str!("fixtures/passive-contact/scene.json"),
            16,
        );
        let peak = result["peak_energy_gain_J"].as_f64().unwrap();
        assert!(peak <= 1.0, "delayed energy gain: {result}");
    }
}
