"""YOLO 물체 검출. 결과를 검출기 공통 출력(Detection)으로 바꿔 돌려준다."""
from .types import Detection


def to_detections(xyxy, confidences, class_ids, names):
    """박스 배열을 Detection 목록으로 바꾼다. 신뢰도가 높은 것부터."""
    detections = []
    for (x1, y1, x2, y2), conf, cls in zip(xyxy, confidences, class_ids):
        cls = int(cls)
        detections.append(Detection(
            name=names[cls], class_id=cls,
            center_uv=((float(x1) + float(x2)) / 2, (float(y1) + float(y2)) / 2),
            bbox_xyxy=(float(x1), float(y1), float(x2), float(y2)),
            confidence=float(conf)))
    return sorted(detections, key=lambda d: d.confidence, reverse=True)


class YoloDetector:
    def __init__(self, model_path, min_confidence=0.5, imgsz=640):
        # ultralytics는 torch까지 끌고 와서 무겁다. YOLO를 쓸 때만 불러온다
        from ultralytics import YOLO
        self.model = YOLO(str(model_path))
        self.names = self.model.names
        self.min_confidence = min_confidence
        self.imgsz = imgsz

    def detect(self, frame):
        result = self.model.predict(frame, conf=self.min_confidence, imgsz=self.imgsz, verbose=False)[0]
        boxes = result.boxes
        return to_detections(boxes.xyxy.tolist(), boxes.conf.tolist(), boxes.cls.tolist(), self.names)
