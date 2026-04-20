use rsruckig::prelude::{
    DataArrayOrVec, DurationDiscretization, InputParameter, OutputParameter, Ruckig,
    RuckigResult, ThrowErrorHandler,
};

#[test]
fn boundary_s_curve_exact_solution_regression() {
    let mut ruckig = Ruckig::<1, ThrowErrorHandler>::new(None, 0.001);
    let mut output = OutputParameter::new(None);

    let input = InputParameter {
        duration_discretization: DurationDiscretization::Discrete,
        current_position: DataArrayOrVec::Stack([4.19948]),
        current_velocity: DataArrayOrVec::Stack([0.0]),
        current_acceleration: DataArrayOrVec::Stack([0.0]),
        target_position: DataArrayOrVec::Stack([3.76748]),
        target_velocity: DataArrayOrVec::Stack([0.0]),
        target_acceleration: DataArrayOrVec::Stack([0.0]),
        max_velocity: DataArrayOrVec::Stack([0.76]),
        max_acceleration: DataArrayOrVec::Stack([2.4]),
        max_jerk: DataArrayOrVec::Stack([8.0]),
        ..InputParameter::new(None)
    };

    match ruckig.update(&input, &mut output) {
        Ok(RuckigResult::Working) | Ok(RuckigResult::Finished) => {}
        other => panic!("expected valid trajectory, got {:?}", other),
    }
}

#[test]
fn boundary_s_curve_exact_solution_multidof_regression() {
    let mut ruckig = Ruckig::<4, ThrowErrorHandler>::new(None, 0.001);
    let mut output = OutputParameter::new(None);

    let input = InputParameter {
        duration_discretization: DurationDiscretization::Discrete,
        current_position: DataArrayOrVec::Stack([4.19948, 1.0399, 90.0, 0.1]),
        current_velocity: DataArrayOrVec::Stack([0.0, 0.0, 0.0, 0.0]),
        current_acceleration: DataArrayOrVec::Stack([0.0, 0.0, 0.0, 0.0]),
        target_position: DataArrayOrVec::Stack([3.76748, 0.57135, 90.0, 0.1]),
        target_velocity: DataArrayOrVec::Stack([0.0, 0.0, 0.0, 0.0]),
        target_acceleration: DataArrayOrVec::Stack([0.0, 0.0, 0.0, 0.0]),
        max_velocity: DataArrayOrVec::Stack([0.76, 0.76, 96.0, 0.4]),
        max_acceleration: DataArrayOrVec::Stack([2.4, 2.4, 640.0, 1.0]),
        max_jerk: DataArrayOrVec::Stack([8.0, 8.0, 1600.0, 25.0]),
        ..InputParameter::new(None)
    };

    match ruckig.update(&input, &mut output) {
        Ok(RuckigResult::Working) | Ok(RuckigResult::Finished) => {}
        other => panic!("expected valid trajectory, got {:?}", other),
    }
}
