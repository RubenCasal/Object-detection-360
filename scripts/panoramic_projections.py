import numpy as np
from stereo import map_to_sphere
import cv2

from concurrent.futures import ThreadPoolExecutor
import torch
import torchvision
from ultralytics.utils.plotting import Annotator, colors


def greedy_merge_largest_box(boxes, scores, iou_thres=0.5):
    """
    boxes: Tensor[N, 4] in (x1, y1, x2, y2)
    scores: Tensor[N] confidences
    Devuelve índices de las cajas seleccionadas tras agrupar por IOU y quedarte con la más grande (por área).
    """
    keep = []
    idxs = torch.argsort(scores, descending=True).tolist()
    visited = set()

    for i in idxs:
        if i in visited:
            continue
        group = [i]
        box_i = boxes[i]

        for j in idxs:
            if j == i or j in visited:
                continue
            box_j = boxes[j]

            # Compute IOU
            inter = (torch.min(box_i[2], box_j[2]) - torch.max(box_i[0], box_j[0])).clamp(0) * \
                    (torch.min(box_i[3], box_j[3]) - torch.max(box_i[1], box_j[1])).clamp(0)
            area_i = (box_i[2] - box_i[0]) * (box_i[3] - box_i[1])
            area_j = (box_j[2] - box_j[0]) * (box_j[3] - box_j[1])
            union = area_i + area_j - inter
            iou = inter / union if union > 0 else 0.0
            
            if iou > iou_thres:
                group.append(j)
                visited.add(j)

        # Elegir la más grande del grupo
        largest = max(group, key=lambda idx: scores[idx] * (boxes[idx][2] - boxes[idx][0]) * (boxes[idx][3] - boxes[idx][1]))        
        keep.append(largest)
        visited.add(i)

    return torch.tensor(keep, dtype=torch.long)


def stereo_bboxes_to_panorama(detections_with_meta, pano_img, stereo_img_size, FOV, model, iou_thres=0.2):

    pano_h, pano_w = pano_img.shape[:2]
    W, H = stereo_img_size
    FOV_W, FOV_H = FOV

    pano_pixel_W_range = int(pano_w * (FOV_W / 360))
    pano_pixel_H_range = int(pano_h * (FOV_H / 180))
    W_step = pano_pixel_W_range / W
    H_step = pano_pixel_H_range / H
    z = pano_pixel_W_range / 4
    radius = z

    projected_boxes = []
    projected_scores = []
    projected_classes = []

    for frame in detections_with_meta:
        yaw = frame['yaw']
        pitch = frame['pitch']
        for det in frame['image_detections']:
            x, y, w, h = det['box']
            conf = det['confidence']
            cls = det['class_id']

            x1 = x - w / 2
            x2 = x + w / 2
            y1 = y - h / 2
            y2 = y + h / 2

            x_plane = np.array([x1, x2]) * W_step - pano_pixel_W_range / 2
            y_plane = pano_pixel_H_range / 2 - np.array([y1, y2]) * H_step

            xg, yg = np.meshgrid(x_plane, y_plane, indexing='xy')

            theta, phi = map_to_sphere(
                xg, yg, z, radius,
                np.radians(yaw),
                np.radians(pitch),
                normalizing=True
            )

            U = (phi * pano_w / (2 * np.pi)).astype(int) % pano_w
            V = (theta * pano_h / np.pi).astype(int)
            V = np.clip(V, 0, pano_h - 1)

            u_min, u_max = U.min(), U.max()
            v_min, v_max = V.min(), V.max()

            if u_max - u_min > pano_w / 2:
                u_coords = U.flatten()
                u_coords[u_coords < pano_w / 2] += pano_w
                u_min = u_coords.min()
                u_max = u_coords.max()
                if u_max >= pano_w:
                    u_min -= pano_w
                    u_max -= pano_w

            x1_pano, x2_pano = np.clip([u_min, u_max], 0, pano_w - 1)
            y1_pano, y2_pano = np.clip([v_min, v_max], 0, pano_h - 1)

            projected_boxes.append([x1_pano, y1_pano, x2_pano, y2_pano])
            projected_scores.append(conf)
            projected_classes.append(cls)

    if not projected_boxes:
        return pano_img, []

    boxes_tensor = torch.tensor(projected_boxes, dtype=torch.float32)
    scores_tensor = torch.tensor(projected_scores, dtype=torch.float32)
    classes_tensor = torch.tensor(projected_classes)

    final_keep = greedy_merge_largest_box(boxes_tensor, scores_tensor, iou_thres=iou_thres)


    for i in final_keep:
        x1, y1, x2, y2 = boxes_tensor[i].int().tolist()
        cls = int(classes_tensor[i])
        conf = scores_tensor[i].item()
        name = model.names[cls] if hasattr(model, "names") else str(cls)
        label = f"{name} {conf:.2f}"
        color = colors(cls, True)
        cv2.rectangle(pano_img, (x1, y1), (x2, y2), color, 2)
        cv2.putText(pano_img, label, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness=1, lineType=cv2.LINE_AA)

    return pano_img, boxes_tensor[final_keep].cpu().numpy()


def apply_stereo_luts_parallel(pano_img, luts, bridge, msg_header, max_workers=4):
    """
    Aplica las LUTs precalculadas en paralelo sobre una imagen panorámica.

    Input:
        pano_img (np.ndarray): imagen panorámica equirectangular (BGR)
        luts (list): salida de precompute_stereo_luts(...)
        bridge (CvBridge): instancia de CvBridge para convertir a sensor_msgs/Image
        msg_header: cabecera original del mensaje de ROS
        max_workers (int): número de hilos paralelos

    Output:
        Lista de sensor_msgs/Image anotadas con el header original
    """

    def remap_and_convert(lut):
        projected_img = cv2.remap(
            pano_img,
            lut['map_U'],
            lut['map_V'],
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_REFLECT
        )
        out_msg = bridge.cv2_to_imgmsg(projected_img, encoding='bgr8')
        out_msg.header = msg_header
        return out_msg

    with ThreadPoolExecutor(max_workers=max_workers) as executor:
        msgs = list(executor.map(remap_and_convert, luts))

    return msgs



def precompute_stereo_luts(pano_shape, output_size, FOV, pitch=90):
    """
    Precompute 4 stereographic projection LUTs with yaw = 0°, 90°, 180°, 270°.

    Input:
        pano_shape (tuple): (height, width) of equirectangular panorama
        output_size (tuple): (W, H) of each projected image
        FOV (tuple): (W_angle, H_angle) in degrees
        pitch (float): default 90° for horizontal strip projections

    Output:
        List of dicts with keys: {'map_U', 'map_V', 'yaw', 'pitch'}
    """
    yaw_angles = [0, 90, 180, 270]
    luts = []

    for yaw in yaw_angles:
        map_U, map_V = generate_lut(pano_shape, output_size, FOV, yaw, pitch)
        luts.append({
            "yaw": yaw,
            "pitch": pitch,
            "map_U": map_U,
            "map_V": map_V
        })

    return luts


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

    map_U = (phi * pano_w / (2 * np.pi)).reshape((H, W)).astype(np.float32)
    map_V = (theta * pano_h / np.pi).reshape((H, W)).astype(np.float32)

    return map_U, map_V

