from perception.sythesis.cone_mapping.cone_mapping.kalman_filter import Cone_Mapper
from fsae_interfaces.msg import ConeMap

def Transformation_test(mapper: Cone_Mapper, msg : ConeMap):
    """Extract measurement state from the Cone Map message subscription

    Args:
        msg: Input ConeMap message from Cone detection

    Returns:
        None

    Raises:
        None
    """
    # Convert Cone Map message into position (x and y), orientation (theta) and list of cones
    x, y, theta, list_of_cones = mapper.convert_message_to_data(msg)
    # Use list of cones and states (x, y and theta) to get the position vector and rotation matrix
    # position_vector, rotation_matrix, list_of_cones = mapper.convert_to_input_matrix(x, y, theta - np.pi / 2, list_of_cones);
    position_vector, rotation_matrix, list_of_cones = mapper.convert_to_input_matrix(x, y, theta, list_of_cones);
    # Conversion from local reference frame to global reference frame
    new_cone_columns = mapper.create_cone_map(position_vector, rotation_matrix, list_of_cones)
    mapper.cone_map_array_measured = new_cone_columns;  # Produce latest measurement

    # Get unsorted Cone Map that contains all measured cone map at moment
    cone_map_measurement_unsorted = mapper.produce_cone_map_message(x, y, theta,
                                                                    mapper.cone_map_array_measured)  # Produce map message
    mapper.publisher.publish(cone_map_measurement_unsorted);
