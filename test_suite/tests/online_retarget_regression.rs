use rsruckig::prelude::{
    DataArrayOrVec, DurationDiscretization, InputParameter, OutputParameter, Ruckig, RuckigResult,
    ThrowErrorHandler,
};

/// Regression test for the online update() loop: when the target changes after a
/// segment finishes (mid-stream retarget), the freshly calculated trajectory must
/// be sampled from its own t=0. The 2.0 error-handling rewrite dropped the
/// `output.time` reset on recalculation, so the new trajectory was sampled at the
/// previous segment's accumulated time — emitting a near-instant jump to the new
/// target (observed in production as a pickup turntable being step-commanded).
#[test]
fn online_retarget_samples_from_trajectory_start() {
    const CYCLE: f64 = 0.004;
    const V_MAX: f64 = 60.0;

    let mut ruckig = Ruckig::<1, ThrowErrorHandler>::new(None, CYCLE);
    let mut input = InputParameter {
        duration_discretization: DurationDiscretization::Discrete,
        current_position: DataArrayOrVec::Stack([176.39]),
        target_position: DataArrayOrVec::Stack([174.39]),
        max_velocity: DataArrayOrVec::Stack([V_MAX]),
        max_acceleration: DataArrayOrVec::Stack([30.0]),
        max_jerk: DataArrayOrVec::Stack([60.0]),
        ..InputParameter::new(None)
    };

    // Segment sequence like the smarkbox pickup-turntable idle wiggle.
    let mut remaining = vec![182.0, 180.0];
    let mut output = OutputParameter::new(None);
    let mut done = false;
    let mut last_pos = input.current_position[0];
    let mut ticks = 0u32;

    loop {
        if done {
            match remaining.is_empty() {
                true => break,
                false => input.target_position = DataArrayOrVec::Stack([remaining.remove(0)]),
            }
        }

        let res = ruckig.update(&input, &mut output).expect("update must succeed");
        assert!(
            matches!(res, RuckigResult::Working | RuckigResult::Finished),
            "unexpected result {res:?}"
        );
        done = res == RuckigResult::Finished;
        output.pass_to_input(&mut input);

        let pos = output.new_position[0];
        let step = (pos - last_pos).abs();
        assert!(
            step <= V_MAX * CYCLE * 1.5,
            "kinematically impossible step of {step} deg in one cycle at tick {ticks}"
        );
        last_pos = pos;

        ticks += 1;
        assert!(ticks < 100_000, "did not converge");
    }

    assert!((last_pos - 180.0).abs() < 1e-6, "did not end at target, got {last_pos}");
    // Full three-segment wiggle takes ~3.6s at these limits; a premature-finish
    // regression completes in well under half that.
    assert!(ticks > 700, "finished suspiciously fast: {ticks} ticks");
}
