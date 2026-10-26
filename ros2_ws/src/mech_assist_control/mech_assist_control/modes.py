"""Valid application control combinations; one strategy per node instance."""


def resolve_mode(strategy, input_mode, output_mode):
    if strategy == 'resistance':
        if input_mode == 'imu' and output_mode == 'preview':
            return 'resistance_preview'
        if input_mode == 'imu' and output_mode == 'motor':
            return 'resistance_motor'
        raise ValueError('resistance requires imu input and preview or motor output')
    if strategy != 'velocity_follow':
        raise ValueError('unknown control strategy')
    if input_mode == 'motor_feedback_simulation':
        if output_mode == 'motor':
            return 'periodic_bench'
        raise ValueError('motor feedback simulation requires motor output')
    if input_mode != 'imu':
        raise ValueError('unknown input mode')
    if output_mode == 'preview':
        return 'velocity_preview'
    if output_mode == 'motor':
        return 'velocity_motor'
    raise ValueError('unknown output mode')
