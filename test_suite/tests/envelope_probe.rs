use rsruckig::prelude::{
    DataArrayOrVec, DurationDiscretization, InputParameter, Ruckig, RuckigResult,
    ThrowErrorHandler, Trajectory,
};

fn run_case(name: &str, p0: [f64; 5], pf: [f64; 5]) {
    let mut ruckig = Ruckig::<5, ThrowErrorHandler>::new(None, 0.001);

    let input = InputParameter {
        duration_discretization: DurationDiscretization::Discrete,
        current_position: DataArrayOrVec::Stack(p0),
        current_velocity: DataArrayOrVec::Stack([0.0; 5]),
        current_acceleration: DataArrayOrVec::Stack([0.0; 5]),
        target_position: DataArrayOrVec::Stack(pf),
        target_velocity: DataArrayOrVec::Stack([0.0; 5]),
        target_acceleration: DataArrayOrVec::Stack([0.0; 5]),
        max_velocity: DataArrayOrVec::Stack([1.14, 1.14, 0.9, 0.4, 180.0]),
        max_acceleration: DataArrayOrVec::Stack([3.0, 3.0, 2.4, 1.0, 1200.0]),
        max_jerk: DataArrayOrVec::Stack([9.0, 9.0, 12.0, 10.0, 3000.0]),
        ..InputParameter::new(None)
    };

    let mut traj = Trajectory::new(None);
    match ruckig.calculate(&input, &mut traj) {
        Ok(RuckigResult::Working) | Ok(RuckigResult::Finished) => {}
        other => panic!("{name}: expected valid trajectory, got {:?}", other),
    }

    let duration = traj.get_duration();
    println!("{name}: duration = {duration}");

    let mut new_pos = DataArrayOrVec::Stack([0.0; 5]);
    let mut new_vel = DataArrayOrVec::Stack([0.0; 5]);
    let mut new_acc = DataArrayOrVec::Stack([0.0; 5]);
    let mut worst: [f64; 5] = [0.0; 5];

    let steps = (duration / 0.001).ceil() as usize;
    for s in 0..=steps {
        let t = (s as f64) * 0.001;
        traj.at_time(
            t.min(duration),
            &mut Some(&mut new_pos),
            &mut Some(&mut new_vel),
            &mut Some(&mut new_acc),
            &mut None,
            &mut Some(0usize),
        );
        for dof in 0..5 {
            let lo = p0[dof].min(pf[dof]);
            let hi = p0[dof].max(pf[dof]);
            let over = (lo - new_pos[dof]).max(new_pos[dof] - hi).max(0.0);
            if over > worst[dof] {
                worst[dof] = over;
            }
        }
    }

    println!("{name}: worst envelope excursion per dof = {:?}", worst);
    for dof in 0..5 {
        assert!(
            worst[dof] < 1e-6,
            "{name}: dof {dof} leaves the [start, target] envelope by {} m",
            worst[dof]
        );
    }
}

#[test]
fn envelope_error_case_1() {
    run_case(
        "case1",
        [3.28318, 1.05031, 0.252, 0.192, 90.0],
        [3.296, 1.05031, 0.444, 0.192, 90.0],
    );
}

#[test]
fn envelope_error_case_2() {
    run_case(
        "case2",
        [2.59056, 1.05132, 0.252, 0.192, 90.0],
        [2.601, 1.05132, 0.444, 0.192, 90.0],
    );
}

#[test]
fn envelope_error_case_3() {
    run_case(
        "case3",
        [1.5391, 1.0387, 0.252, 0.055, 90.0],
        [1.555, 1.0387, 0.444, 0.055, 90.0],
    );
}

#[test]
fn envelope_sweep() {
    // Sweep of x-distances against the fixed boundary z-move that triggers
    // the degenerate a_max-grazing profile (pd=0.192, j=12 -> t=0.2, a_peak=2.4).
    for i in 0..200 {
        let dx = 0.001 + (i as f64) * 0.005;
        run_case(
            &format!("sweep dx={dx}"),
            [0.0, 1.0, 0.252, 0.192, 90.0],
            [dx, 1.0, 0.444, 0.192, 90.0],
        );
    }
}
