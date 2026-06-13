#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D
from bt_policy.srv import GetDetections
from cv_bridge import CvBridge
import torch
from torchvision.models.detection.faster_rcnn import fasterrcnn_resnet50_fpn, FastRCNNPredictor
from torchvision.transforms import functional as F
from PIL import Image as PilImage
import cv2
import numpy as np


detected_objects_topic = '/bluerov/camera/detected_objects'
camera_image_topic = '/bluerov/camera/image_color'
get_objects_service = '/bluerov/camera/get_detected_objects'
model_name_anode_shark = 'anode_shark_model_augmented.pth'
model_name_anode = 'anode_model.pth'

def save_annotated_image(cv_image, detections_msg, save_dir='detections_output'):
    """
    Annotate and save image with bounding boxes and labels.
    
    Args:
        cv_image: OpenCV BGR image (numpy array)
        detections_msg: Detection2DArray message
        save_dir: Directory to save images
    """
    os.makedirs(save_dir, exist_ok=True)
    
    # Create copy to avoid modifying original
    annotated = cv_image.copy()
    
    for detection in detections_msg.detections:
        bbox = detection.bbox
        x_center = bbox.center.position.x
        y_center = bbox.center.position.y
        width = bbox.size_x
        height = bbox.size_y
        
        # Convert center+size to corner coordinates
        x1 = int(x_center - width / 2)
        y1 = int(y_center - height / 2)
        x2 = int(x_center + width / 2)
        y2 = int(y_center + height / 2)
        
        if detection.results:
            class_id = detection.results[0].hypothesis.class_id
            score = detection.results[0].hypothesis.score
            
            # Color coding: anode=green, fish=red
            color = (0, 255, 0) if class_id == 'anode' else (0, 0, 255)
            
            # Draw box and label
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            label = f"{class_id} {score:.2f}"
            cv2.putText(annotated, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    # Save with timestamp
    timestamp = detections_msg.header.stamp.sec + detections_msg.header.stamp.nanosec * 1e-9
    filename = f"detection_{timestamp:.3f}.jpg"
    filepath = os.path.join(save_dir, filename)
    cv2.imwrite(filepath, annotated)
    
    return filepath

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.subscription = self.create_subscription(
            Image,
            camera_image_topic,
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Detection2DArray, detected_objects_topic, 10)
        self.bridge = CvBridge()
        self.conf_thresh = 0.5
        
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        
        model_anode_shark_path = os.path.join(os.path.dirname(__file__), 'models', model_name_anode_shark)
        model_anode_path = os.path.join(os.path.dirname(__file__), 'models', model_name_anode)
        print(f"taking the model from path: {model_anode_path} and {model_anode_shark_path}")
        self.class_names_anode_shark = ['__background__', 'anode', 'fish']
        self.class_names_anode = ['__background__', 'anode']


        self.model_anode_shark = self._load_model(model_anode_shark_path, len(self.class_names_anode_shark))
        self.get_logger().info(f'Loaded primary model (anode+fish) from {model_anode_shark_path}')

        self.model_anode = self._load_model(model_anode_path, len(self.class_names_anode))
        self.get_logger().info(f'Loaded primary model (anode) from {model_anode_path}')

        self.cb_group = ReentrantCallbackGroup()
        self.latest_image = None
        self.latest_header = None
        self.detection_service = self.create_service(
            GetDetections,
            get_objects_service,
            self.detection_callback,
            callback_group=self.cb_group
        )
    
    def _load_model(self, model_path, num_classes):
        """Load and return a Faster R-CNN model."""
        model = fasterrcnn_resnet50_fpn(weights=None, num_classes=num_classes)
        in_features = model.roi_heads.box_predictor.cls_score.in_features
        model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
        model.load_state_dict(torch.load(model_path, map_location=self.device))
        model.to(self.device)
        model.eval()
        self.get_logger().info(f'Loaded model from {model_path}')
        return model
    
    def _run_inference(self, img_tensor, model, class_names, model_name):
        """Run inference on image tensor with given model."""
        detections_msg = Detection2DArray()
        
        with torch.no_grad():
            outputs = model(img_tensor)[0]
        
        boxes = outputs['boxes'].cpu().numpy()
        scores = outputs['scores'].cpu().numpy()
        labels = outputs['labels'].cpu().numpy()
        
        self.get_logger().info(f'{model_name} model: {len(boxes)} raw predictions')
        for i, (box, score, label) in enumerate(zip(boxes, scores, labels)):
                self.get_logger().info(
                    f'  [{i}] {class_names[label]}: {score:.3f} at '
                    f'[{box[0]:.1f}, {box[1]:.1f}, {box[2]:.1f}, {box[3]:.1f}]'
                )
        
        for box, score, label in zip(boxes, scores, labels):
            if score < self.conf_thresh:
                continue
            detection = Detection2D()
            detection.bbox = BoundingBox2D()
            detection.bbox.center.position.x = float((box[0] + box[2]) / 2)
            detection.bbox.center.position.y = float((box[1] + box[3]) / 2)
            detection.bbox.size_x = float(box[2] - box[0])
            detection.bbox.size_y = float(box[3] - box[1])
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_names[label]
            hypothesis.hypothesis.score = float(score)
            detection.results.append(hypothesis)
            detections_msg.detections.append(detection)
        
        return detections_msg

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.latest_image = cv_image
        self.latest_header = msg.header
        self.get_logger().info('Captured latest image', throttle_duration_sec=30)

    def detection_callback(self, request, response):
        start_time = self.get_clock().now()
        detections_msg = Detection2DArray()
        if self.latest_image is None:
                response.detections = detections_msg
                response.success = False
                return response
        
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < 1.0:
            pil_image = PilImage.fromarray(cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2RGB))
            img_tensor = F.to_tensor(pil_image).unsqueeze(0).to(self.device)
            detections_msg = self._run_inference(img_tensor, self.model_anode_shark, self.class_names_anode_shark, "primary")

            if len(detections_msg.detections) == 0:
                self.get_logger().info('Primary model found nothing, trying fallback (anode only)...')
                detections_msg = self._run_inference(img_tensor, self.model_anode, self.class_names_anode, "fallback")
            detections_msg.header = self.latest_header
            if len(detections_msg.detections) > 0:
                self.publisher.publish(detections_msg)
                self.get_logger().info(f'Detected {len(detections_msg.detections)} objects')
                break
            self.get_logger().info(f'detecting...')
        saved_path = save_annotated_image(self.latest_image, detections_msg)
        self.get_logger().info(f'saved image in {saved_path}')
        response.detections = detections_msg
        response.success = True
        self.get_logger().info(f'Service done: {len(detections_msg.detections)} final detections')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
