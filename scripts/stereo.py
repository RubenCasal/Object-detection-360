
import numpy as np
import cv2
from scipy.ndimage import map_coordinates
from draw_boxes import draw_bounding_box


'''

Coordinate System:
    coordinate system is defined as:
    x -> south-direction axis
    y -> zenith-direction axis
    z -> west-direction axis
    theta -> zenith angle from positive y-axis
    phi -> azimuth angle from positive z-axis

'''


def generate_lut(pano_shape, output_size, FOV, yaw, pitch):
    pano_h, pano_w = pano_shape
    W, H = output_size

    pano_pixel_W_range = int(pano_w * (FOV[0] / 360))
    pano_pixel_H_range = int(pano_h * (FOV[1] / 180))
    W_step_size = pano_pixel_W_range / W
    H_step_size = pano_pixel_H_range / H

    u, v = np.rint(np.meshgrid(
        np.arange(pano_pixel_W_range, step=W_step_size),
        np.arange(pano_pixel_H_range, step=H_step_size),
        indexing='xy'))

    x = u - pano_pixel_W_range / 2
    y = pano_pixel_H_range / 2 - v
    z = pano_pixel_W_range / 4
    radius = z

    theta, phi = map_to_sphere(x, y, z, radius, np.radians(yaw), np.radians(pitch))

    map_U = (phi * pano_w / (2 * np.pi)).astype(np.float32)
    map_V = (theta * pano_h / np.pi).astype(np.float32)

    return map_U, map_V



def map_to_sphere(x, y, z, radius, yaw_radian, pitch_radian, normalizing=True):
    
    '''
    Function:
        Take input coordinates (x, y, z)
        and find what coordinates on the sphere project
        onto them using stereographic projection.

        This means finding the points on the sphere defined by:
            x^2 + y^2 + z^2 = radius^2
        that are along the vector line passing through the points:
            (0, 0, -R) and (x, y, z)

    Input:
        x, y, z (np.ndarray, np.ndarray, np.ndarray): 3 np.ndarrays input point coordinates
        radius (float): input radius of the sphere (this sphere is what will be used to wrap the panorama around)
        yaw_radians (float): how much to rotate around y axis
        pitch_radians (float): how much to rotate around z axis

    Output:
        theta, phi: 2 np.ndarrays of coordinates in (theta, phi) to use
        to map pixel colors from the panorama to the pixels in the output image
    '''

    r = radius

    # Equations to get the original point before projected onto plane
    # This point is the intersection of the line through the points (0, 0, -R) (x_p, y_p, z_p) and the sphere x^2 + y^2 + z^2 = r^2
    denominator = x ** 2 + y ** 2 + 4 * (r ** 2)
    x_circle = (4 * r ** 2) * x / denominator
    y_circle = (4 * r ** 2) * y / denominator
    z_circle = -r + (8 * r ** 3) / denominator

    # Calculate theta (azimuth angle) and phi (zenith angle) 
    theta = np.arccos(z_circle / np.sqrt(x_circle ** 2 + y_circle ** 2 + z_circle ** 2))
    phi = np.arctan2(y_circle, x_circle)


    # Apply rotation transformations here
    theta_prime = np.arccos(np.sin(theta) * np.sin(phi) * np.sin(pitch_radian) +
                            np.cos(theta) * np.cos(pitch_radian))

    phi_prime = np.arctan2(np.sin(theta) * np.sin(phi) * np.cos(pitch_radian) -
                           np.cos(theta) * np.sin(pitch_radian),
                           np.sin(theta) * np.cos(phi))
    phi_prime += yaw_radian
    if normalizing:
        phi_prime = phi_prime % (2 * np.pi)

    return theta_prime.flatten(), phi_prime.flatten()


def interpolate_color(coords, img, method='bilinear'):
    '''
    Function:
        Take input coordinates and use the input image to
        get the color values at each coordinate
        (interpolate color values between
        coordinate values if necessary).

    Input:
        coords (np.ndarray): a np.ndarray of coordinates
        img (np.ndarray): a np.ndarray of image color values
        method (str/int: optional): type of interpolation

    Output:
        np.ndarray of color values of each related coordinate,
        and an array shape of coords
    '''
    order = {'nearest': 0, 'bilinear': 1, 'bicubic': 3}.get(method, 1)
    red = map_coordinates(img[:, :, 0], coords, order=order, mode='reflect')
    green = map_coordinates(img[:, :, 1], coords, order=order, mode='reflect')
    blue = map_coordinates(img[:, :, 2], coords, order=order, mode='reflect')

    return np.stack((red, green, blue), axis=-1)


def panorama_to_plane(pano_array, FOV, output_size, yaw, pitch):
    '''
    Function:
        Take an input panorama image (360-degree),
        a set Field of View, a file size in pixels-by-pixels for the output plane image,
        and the rotational transformation yaw and pitch and get the
        stereographic projection onto a plane tangeant to the sphere
        created by the panorama.

    Input:
        pano_array (np.ndarray): panorama pixel values arranged in an np.ndarray with shape (pano_height, pano_width, 3) -> 3 for RGB.
        FOV (float, float): (width-angle, height-angle) -> (based on theta from 0 to 360, based on phi from 0 to 180)
        output_size (int, int): (width-pixels, height-pixels)
        yaw_radians (float): how much to rotate around y axis
        pitch_radians (float): how much to rotate around z axis

    Output:
        output_image_array: np.ndarray of color values over the output plane
        image with shape (height, width, 3): 3 for Red, Green, Blue
    '''

    # Get info about its size in pixels
    pano_width = pano_array.shape[1]
    pano_height = pano_array.shape[0]

    # Convert to radians for numpy math
    yaw_radian = np.radians(yaw)
    pitch_radian = np.radians(pitch)

    # Get FOV angles and size of output in pixels
    Panorama_W_angle, Panorama_H_angle = FOV
    W, H = output_size

    # Number of pixels covered by FOV angle in the panorama image
    pano_pixel_W_range = int(pano_width * (Panorama_W_angle / 360))
    pano_pixel_H_range = int(pano_height * (Panorama_H_angle / 180))
    

    # Step size for how much each output pixel changes relative to panorama pixel range
    W_step_size = pano_pixel_W_range / W
    H_step_size = pano_pixel_H_range / H 

    # Create arrays of length W and H, with values from 1 to W and 1 to H
    u, v = np.rint(np.meshgrid(np.arange(pano_pixel_W_range, step=W_step_size), np.arange(pano_pixel_H_range, step=H_step_size), indexing='xy'))

    # Set array of x values from -W/2 to W/2 so that 0 is in center
    x = u - pano_pixel_W_range / 2
    # Set array of y values from -H/2 to H/2 so that 0 is in center
    y = pano_pixel_H_range / 2 - v
    # z distance of plane to center of sphere (plane is parallel to xy-plane and assuming we will be using 180-degree stereographic projection)
    z = pano_pixel_W_range / 4
    # The radius of the sphere is the same distance as the distance of the plane from the xy plane
    radius = z

    # Get an np.ndarray of theta and phi angles that are associated with each point (x, y, z) in the stereographic projected plane
    theta, phi = map_to_sphere(x, y, z, radius, yaw_radian, pitch_radian)

    # Convert theta and phi angles to associated pixels in the panorama
    U = phi * pano_width / (2 * np.pi)
    V = theta * pano_height / np.pi

    # Format the pixels appropriately
    U, V = U.flatten(), V.flatten()
    coords = np.vstack((V, U))

    # Get the pixel values in the for the coordinates of (theta, phi) associated with the stereographic projected plane
    output_image_array = interpolate_color(coords, pano_array)

    # reshape the output to be in the format of the output image
    output_image_array = output_image_array.reshape((H, W, 3)).astype('uint8')

    return output_image_array

def panorama_to_stereo_multiprojections(pano_array, stereographic_image_size, FOV):
    '''
    Function:
        Take a panorama image and convert it to 4 stereographic projections,
        based on "Object Detection in Equirectangular Panorama" paper.

    Input:
        pano_array (np.ndarray): panorama pixel values arranged in an np.ndarray with shape (pano_height, pano_width, 3) -> 3 for RGB.
        FOV (float, float): (width-angle, height-angle) -> (based on theta from 0 to 360, based on phi from 0 to 180)
        ou:
        frames_with_meta_np: a np.ndarray with 4 tuples for each projection:
            (1) an np.ndarray of a stereographic projection plane with shape:
            (output_image_size.height, output_image_size.width, 3) -> 3 for Red, Green, Blue
            (2) its associated yaw and pitch rotations
    '''

    frames_with_meta = []

    for i in range(4):
        yaw_rotation = i * 90
        pitch_rotation = 90
        W, H = stereographic_image_size
        output_image_array = panorama_to_plane(pano_array, FOV, stereographic_image_size, yaw_rotation, pitch_rotation)

        frame_with_meta = (output_image_array, yaw_rotation, pitch_rotation)
        frames_with_meta.append(frame_with_meta)

        '''
        For testing purposes to show the output stereographic projected image
        '''
        # reformatted_output = output_image_array.reshape((H, W, 3)).astype('uint8')
        # cv2.imshow("output image", reformatted_output)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()
    

    frames_with_meta_np = np.array(frames_with_meta, dtype=np.dtype([('image', np.ndarray), ('yaw', int), ('pitch', int)]))

    return frames_with_meta_np

def calculate_IoU(box1, box2):
    '''
    Function:
        Take two boxes and calculate the intersection-over-union value
    
    Input:
        box1 (int/float, int/float, int/float, int/float): (x, y, w, h) of first box
        box2 (int/float, int/float, int/float, int/float): (x, y, w, h) of second box

    Output:
        IoU (float): intersection-over-union value
    '''
    # Extract coordinates of the bounding boxes
    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2

    # Calculate the intersection rectangle
    intersection_x = max(x1, x2)
    intersection_y = max(y1, y2)
    intersection_w = max(0, min(x1 + w1, x2 + w2) - intersection_x)
    intersection_h = max(0, min(y1 + h1, y2 + h2) - intersection_y)

    # Calculate the areas of each rectangle
    area_box1 = w1 * h1
    area_box2 = w2 * h2
    area_intersection = intersection_w * intersection_h

    # Calculate the IoU
    IoU = area_intersection / float(area_box1 + area_box2 - area_intersection)

    return IoU

def soft_nms(pano_detections_with_meta, sigma_one=0.3, sigma_two=0.6, IoU_minimum=0.5):
    '''
    Function:
        Take in all panorama detections and filter out those based on the soft NMS penalty described in the paper
    
    Input:
        pano_detections_with_meta (np.ndarray): Contains all panorama detections
        sigma_one (float from 0 to 1): first parameter described in paper
        sigma_two (float from 0 to 1): second parameter described in paper
        IoU_minimum (float from 0 to 1): the minimum IoU needed to apply soft NMS between two detections

    Output:
        pano_detections_post_nms: an np.ndarray containing remaining panorama detections after soft NMS is applied
    '''

    pano_detections_post_nms = []
    indices_to_remove = set()

    # For each panorama detection, if it overlaps a lot with another detection (thus duplicate detections),
    # calculate the soft NMS penalty described in paper
    num_detections = pano_detections_with_meta.shape[0]
    for detection_i in range(num_detections):
        if detection_i in indices_to_remove:
            pass
        detection = pano_detections_with_meta[detection_i]['pano_detection']

        for detection_j in range(detection_i + 1, num_detections):
            compared_detection = pano_detections_with_meta[detection_j]['pano_detection']

            # Check if the detections are for the same class
            if detection['class_id'] == compared_detection['class_id']:
                # Calculate the IoU of the boxes
                IoU = calculate_IoU(detection['box'], compared_detection['box'])
                # Check if IoU is above the minimum threshold
                if IoU >= IoU_minimum:
                    confidence = detection['confidence']
                    distance = detection['dist_from_center']
                    # Based on equation in paper
                    rescore = confidence * np.exp(-1 * ((IoU ** 2)/sigma_one + (distance ** 2)/sigma_two))

                    compared_confidence = compared_detection['confidence']
                    compared_distance = compared_detection['dist_from_center']
                    compared_rescore = compared_confidence * np.exp(-1 * ((IoU ** 2)/sigma_one + (compared_distance ** 2)/sigma_two))

                    '''
                    # Print for testing
                    print("detection: ", detection)
                    print("compared detection: ", compared_detection)
                    print("rescore for detection: ", rescore)
                    print("rescore for compared detection: ", compared_rescore)
                    '''

                    # If detection is better than compared, then add compared to discard pile
                    if rescore >= compared_rescore:
                        indices_to_remove.add(detection_j)
                    # Otherwise, add detection to the discard pile, finish inner loop, and begin checking the next detection
                    else:
                        indices_to_remove.add(detection_i)
                        break
            
        if detection_i not in indices_to_remove:
            pano_detections_post_nms.append(detection)


    pano_detections_post_nms = np.array(pano_detections_post_nms, dtype=np.dtype([('pano_detection', object)]))

    return pano_detections_post_nms

def stereo_bounding_boxes_to_panorama(frame_detections_with_meta, panorama, stereographic_image_size, FOV):

    '''
    Function:
        Take a set of detections on the stereographic projections and
        convert them to coordinates on the panorama, then return the
        panorama with detections on it.

    Input:
        detections_with_meta (np.ndarray of tuple with (np.ndarray, (int, int), int, int)):
            (bounding boxes, (stereographic image width, height), yaw, pitch))
            -> bounding boxes info (see yolov8 model run), the height and width in pixels
            of the image used in detections, and the yaw and pitch of frame used in detections. 
        pano_array (np.ndarray): panorama pixel values arranged in an np.ndarray with shape (pano_height, pano_width, 3) -> 3 for RGB.
        stereographic_image_size (int, int): (pixel_W_size, pixel_H_size)
        FOV (int, int): (W_angle, H_angle)
    
    Output:
        annotated_panorama (np.ndarray): The output panorama with annotated bounding boxes
    '''

    # Create copy of panorama to avoid overwriting values on 'panorama'
    pano_array = np.copy(panorama)

    # Get the size of the panorama in pixels
    pano_width = pano_array.shape[1]
    pano_height = pano_array.shape[0]

    Panorama_W_angle, Panorama_H_angle = FOV
    W, H = stereographic_image_size

    # Number of pixels covered by FOV angle in the panorama image
    pano_pixel_W_range = int(pano_width * (Panorama_W_angle / 360))
    pano_pixel_H_range = int(pano_height * (Panorama_H_angle / 180))
    

    # Step size for how much each output pixel changes relative to panorama pixel range
    W_step_size = pano_pixel_W_range / W
    H_step_size = pano_pixel_H_range / H


    # z distance of plane to center of sphere (plane is parallel to xy-plane and assuming we will be using 180-degree stereographic projection)
    z = pano_pixel_W_range / 4
    # The radius of the sphere is the same distance as the distance of the plane from the xy plane
    radius = z

    pano_detections_with_meta = []

    # Iterate through each frame and convert stereographic coordinates to panorama coordinates
    for frame_index in range(frame_detections_with_meta.shape[0]):
        frame_detections = frame_detections_with_meta[frame_index]

        # Get all of the detections in the frame and get the coordinates of the 4 corners of the box
        for detection_index in range(len(frame_detections['image_detections'])):
            detection = frame_detections['image_detections'][detection_index]
            # box -> {x, y, width, height}
            box = detection['box']
            # scale is conversion factor to scale up to original image (stereographic projection plane) size
            # from downscaled YOLOv8 image (needed to process)
            scale = detection['scale']

            # Get box coordinates for x (x, x + w) and y (y, y + h) from the stereographic image
            box_coord_x = np.array([(box[0] * scale), (box[0] + box[2]) * scale])
            box_coord_y = np.array([(box[1] * scale), (box[1] + box[3]) * scale])

            # Get the stereographic projection plane coordinates from the stereographic image
            # Set array of x values from -W/2 to W/2 so that 0 is in center of plane
            x_plane = box_coord_x * (W_step_size) - pano_pixel_W_range / 2

            # Set array of y values from -H/2 to H/2 so that 0 is in center of plane
            y_plane = pano_pixel_H_range / 2 - box_coord_y * (H_step_size)

            # Get distance from frame center to use for soft NMS described in paper
            plane_center_coord_x = (x_plane[0] + x_plane[1]) / 2
            plane_center_coord_y = (y_plane[0] + y_plane[1]) / 2
            distance_from_frame_center = (np.sqrt(plane_center_coord_x ** 2 + plane_center_coord_y ** 2)
                                          / np.sqrt((pano_pixel_W_range / 2) ** 2 + (pano_pixel_H_range / 2) ** 2))


            x_grid, y_grid = np.meshgrid(x_plane, y_plane, indexing="xy")
            
            # Get associated theta and phi angles
            theta, phi = map_to_sphere(x_grid, y_grid, z, radius, np.radians(frame_detections['yaw']), np.radians(frame_detections['pitch']), normalizing=True)

            # Convert theta and phi angles to associated pixels in the panorama
            U = phi * pano_width / (2 * np.pi)
            V = theta * pano_height / np.pi

            # Correct values for objects that are on the edge of the pano
            if U[2] < (U[0] - pano_width / 2):
                U[2] += pano_width
            elif U[0] < (U[2] - pano_width / 2):
                U[0] += pano_width

            if V[1] < (V[0] - pano_height / 2):
                V[1] += pano_height
            elif V[0] < (V[1] - pano_height / 2):
                V[0] += pano_height

            if U[1] < U[0] - pano_width / 4 or U[1] < U[2] - pano_width / 4:
                U[1] += pano_width
            if U[3] < U[0] - pano_width / 4 or U[3] < U[2] - pano_width / 4:
                U[3] += pano_width
            if V[2] < V[0] - pano_height / 4 or V[2] < V[1] - pano_height / 4:
                V[2] += pano_height
            if V[3] < V[0] - pano_height / 4 or V[3] < V[1] - pano_height / 4:
                V[2] += pano_height

            # Take the smallest x and y value for top left corner and largest x and y value for the bottom right corner
            x_pano = min([U[0], U[2]])
            y_pano = min([V[0], V[1]])
            x_plus_w_pano = max([U[1], U[3]])
            y_plus_h_pano = max([V[2], V[3]])

            # Round to nearest integer for convert for pixels
            x_pano = round(x_pano)
            y_pano = round(y_pano)
            x_plus_w_pano = round(x_plus_w_pano)
            y_plus_h_pano = round(y_plus_h_pano)
            w_pano = x_plus_w_pano - x_pano
            h_pano = y_plus_h_pano - y_pano
            

            pano_detection = detection.copy()

            # Box is defined as (x, y, width, height), where x and y are the top left corner of the box, keeping in line with YOLOv8 box definitions
            pano_detection['box'] = [x_pano, y_pano, w_pano, h_pano]
            pano_detection['dist_from_center'] = distance_from_frame_center

            pano_detections_with_meta.append(pano_detection)
    
    pano_detections_with_meta = np.array(pano_detections_with_meta, dtype=np.dtype([('pano_detection', object)]))


    # Apply Soft Non-Max Suppression on each detection
    pano_detections_post_nms = soft_nms(pano_detections_with_meta)

    # For each panorama detection, draw the corresponding bounding box with the associated pixels on the panorama
    for frame_index in range(pano_detections_post_nms.shape[0]):
        pano_detection = pano_detections_post_nms[frame_index]['pano_detection']


        # box -> {x, y, width, height}
        box = pano_detection['box']

        # Get bounding box information
        x_pano = box[0]
        x_plus_w_pano = box[0] + box[2]
        y_pano = box[1]
        y_plus_h_pano = box[1] + box[3]

        # Get label info from detection
        class_id = pano_detection['class_id']
        confidence = pano_detection['confidence']

        # Draw the bounding box on the panorama
        draw_bounding_box(
            pano_array,
            class_id,
            confidence,
            x_pano,
            y_pano,
            x_plus_w_pano,
            y_plus_h_pano,
        )


    # Return the final annotated pano_array
    return pano_array, pano_detections_post_nms