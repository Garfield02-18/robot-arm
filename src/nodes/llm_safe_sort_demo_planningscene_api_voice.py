#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import re
import threading
import yaml
import rospy
import numpy as np
import cv2
import smach
import actionlib
import moveit_commander
import requests
import json
import time
import os
import urllib.request
from geometry_msgs.msg import PoseStamped
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from sagittarius_object_color_detector.msg import SGRCtrlAction, SGRCtrlGoal, SGRCtrlResult


# ============================
# 模型自动下载工具函数
# ============================

def download_yolo_model(model_path, model_url=None):
    """
    自动下载 YOLO-World 模型文件
    Args:
        model_path: 模型保存路径
        model_url: 下载链接，默认使用 Ultralytics 官方 v8.3.0 版本
    Returns:
        bool: 是否成功
    """
    if model_url is None:
        # 默认使用 Ultralytics 官方 releases
        # 根据文件名推断 URL
        filename = os.path.basename(model_path)
        model_url = f"https://github.com/ultralytics/assets/releases/download/v8.3.0/{filename}"
    
    # 确保目录存在
    model_dir = os.path.dirname(model_path)
    if model_dir and not os.path.exists(model_dir):
        try:
            os.makedirs(model_dir, exist_ok=True)
            rospy.loginfo(f"[Download] 创建目录: {model_dir}")
        except Exception as e:
            rospy.logerr(f"[Download] 创建目录失败: {e}")
            return False
    
    rospy.loginfo(f"[Download] 开始下载模型...")
    rospy.loginfo(f"[Download] 来源: {model_url}")
    rospy.loginfo(f"[Download] 目标: {model_path}")
    
    try:
        # 使用 urllib 下载，显示进度
        def report_hook(block_num, block_size, total_size):
            downloaded = block_num * block_size
            percent = min(100, downloaded * 100 / total_size) if total_size > 0 else 0
            if block_num % 20 == 0:  # 每 20 个 block 打印一次，避免日志过多
                rospy.loginfo(f"[Download] 进度: {percent:.1f}% ({downloaded / 1024 / 1024:.1f} MB / {total_size / 1024 / 1024:.1f} MB)")
        
        urllib.request.urlretrieve(model_url, model_path, reporthook=report_hook)
        
        # 验证文件是否存在且不为空
        if os.path.exists(model_path) and os.path.getsize(model_path) > 0:
            file_size_mb = os.path.getsize(model_path) / 1024 / 1024
            rospy.loginfo(f"[Download] 下载完成! 文件大小: {file_size_mb:.1f} MB")
            return True
        else:
            rospy.logerr("[Download] 下载后文件验证失败")
            return False
            
    except Exception as e:
        rospy.logerr(f"[Download] 下载失败: {e}")
        # 清理可能的不完整文件
        if os.path.exists(model_path):
            try:
                os.remove(model_path)
            except:
                pass
        return False


# ============================
# YAML 与几何工具函数
# ============================

def load_yaml(filename):
    with open(filename, "r") as f:
        return yaml.safe_load(f)

def pixel_to_robot(cx, cy, kb):
    return kb['k1'] * cy + kb['b1'], kb['k2'] * cx + kb['b2']

def in_workspace(x, y):
    return 0.12 <= x <= 0.30 and -0.18 <= y <= 0.18

def find_nearest(object_list, px, py):
    best, dist = None, 1e9
    for obj in object_list:
        d = math.hypot(obj["robot_x"] - px, obj["robot_y"] - py)
        if d < dist:
            best, dist = obj, d
    return best, dist


def infer_object_classes_from_tasks(task_rules):
    """
    根据解析后的任务规则，动态推断需要检测的 YOLO-World 类别
    返回: 类别列表，用于 set_classes()
    """
    classes = set()
    
    for task in task_rules:
        obj_name = task.get("object_name", "block")
        
        # 映射到 YOLO-World 可识别的词汇
        if obj_name in ["block", "cube"]:
            # 对于 block，需要检测基础 block 以及带颜色的 block，让 HSV 去区分颜色
            classes.add("block")
            classes.add("cube")
            # 也可以加上带颜色的，让 YOLO 直接尝试检测
            classes.add("red block")
            classes.add("green block")
            classes.add("blue block")
        elif obj_name in ["banana"]:
            classes.add("banana")
        elif obj_name in ["cup", "mug"]:
            classes.add("cup")
            classes.add("mug")
        elif obj_name != "any":
            # 其他具体物体名称直接加入
            classes.add(obj_name)
    
    # 保底：如果解析结果中没有具体物体，至少保留 block 类
    if not classes:
        classes.add("block")
        classes.add("cube")
    
    return list(classes)


def build_hsv_mask(hsv_img, lower, upper):
    if int(lower[0]) > int(upper[0]):
        lower1 = np.array([0, lower[1], lower[2]], dtype=np.uint8)
        upper1 = np.array([upper[0], upper[1], upper[2]], dtype=np.uint8)
        lower2 = np.array([lower[0], lower[1], lower[2]], dtype=np.uint8)
        upper2 = np.array([180, upper[1], upper[2]], dtype=np.uint8)
        return cv2.add(
            cv2.inRange(hsv_img, lower1, upper1),
            cv2.inRange(hsv_img, lower2, upper2),
        )
    return cv2.inRange(hsv_img, lower, upper)


def extract_color_from_text(text):
    normalized = (text or "").strip().lower().replace("-", " ").replace("_", " ")
    if "red" in normalized or "\u7ea2" in normalized:
        return "red"
    if "green" in normalized or "\u7eff" in normalized:
        return "green"
    if "blue" in normalized or "\u84dd" in normalized:
        return "blue"
    return "any"


def normalize_object_name(name):
    normalized = (name or "").strip().lower().replace("-", " ").replace("_", " ")
    normalized = re.sub(r"\s+", " ", normalized)
    if not normalized or normalized == "any":
        return "any"
    if "banana" in normalized or "\u9999\u8549" in normalized:
        return "banana"
    if (
        "cup" in normalized
        or "mug" in normalized
        or "\u676f\u5b50" in normalized
        or "\u6c34\u676f" in normalized
        or "\u676f" in normalized
    ):
        return "cup"
    if (
        "block" in normalized
        or "cube" in normalized
        or "\u65b9\u5757" in normalized
        or "\u79ef\u6728" in normalized
    ):
        return "block"
    return normalized.replace(" ", "_")


def infer_object_name(text, color="any"):
    object_name = normalize_object_name(text)
    if object_name != "any":
        return object_name
    if color != "any":
        return "block"
    return "any"


def normalize_task_rule(task):
    normalized = dict(task)
    normalized["color"] = normalized.get("color", "any")
    normalized["position"] = normalized.get("position", "any")
    normalized["place_name"] = normalized.get("place_name", "A")
    normalized["raw"] = normalized.get("raw", "")
    normalized["object_name"] = normalized.get(
        "object_name",
        infer_object_name(normalized["raw"], normalized["color"])
    )
    normalized["object_name"] = infer_object_name(
        normalized["object_name"], normalized["color"]
    )
    return normalized


def normalize_yolo_class(cls_name):
    label = (cls_name or "").strip().lower().replace("-", "_").replace(" ", "_")
    object_name = normalize_object_name(cls_name)
    color = extract_color_from_text(cls_name)
    if object_name == "block" and color != "any":
        label = "%s_block" % color
    elif not label:
        label = object_name
    return label, object_name, color


def classify_color_in_bbox(img, hsv_cfg, bbox):
    h, w = img.shape[:2]
    x1, y1, x2, y2 = [int(round(v)) for v in bbox]
    x1 = max(0, min(w - 1, x1))
    y1 = max(0, min(h - 1, y1))
    x2 = max(x1 + 1, min(w, x2))
    y2 = max(y1 + 1, min(h, y2))
    roi = img[y1:y2, x1:x2]
    if roi.size == 0:
        return "any"
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    best_color = "any"
    best_score = 0
    roi_area = roi.shape[0] * roi.shape[1]
    min_score = max(30, int(roi_area * 0.02))
    for color, cfg in hsv_cfg.items():
        mask = build_hsv_mask(hsv_roi, cfg["lower"], cfg["upper"])
        score = int(cv2.countNonZero(mask))
        if score > best_score:
            best_color = color
            best_score = score
    if best_score < min_score:
        return "any"
    return best_color


def bbox_iou(box_a, box_b):
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    inter_x1 = max(ax1, bx1)
    inter_y1 = max(ay1, by1)
    inter_x2 = min(ax2, bx2)
    inter_y2 = min(ay2, by2)
    inter_w = max(0.0, inter_x2 - inter_x1)
    inter_h = max(0.0, inter_y2 - inter_y1)
    inter_area = inter_w * inter_h
    if inter_area <= 0.0:
        return 0.0
    area_a = max(1.0, (ax2 - ax1) * (ay2 - ay1))
    area_b = max(1.0, (bx2 - bx1) * (by2 - by1))
    return inter_area / float(area_a + area_b - inter_area)


def detect_all_hsv(img, hsv_cfg, kb, min_area=1500, show_debug=False):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    debug_img = img.copy()
    all_objs = []

    for color, cfg in hsv_cfg.items():
        mask = build_hsv_mask(hsv_img, cfg["lower"], cfg["upper"])
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = float(cv2.contourArea(c))
            if area < min_area:
                continue
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)
            cx = float(np.mean(box[:, 0]))
            cy = float(np.mean(box[:, 1]))
            x, y, w, h = cv2.boundingRect(c)
            rx, ry = pixel_to_robot(cx, cy, kb)
            obj = {
                "source": "hsv",
                "class": "%s_block" % color,
                "label": "%s_block" % color,
                "object_name": "block",
                "color": color,
                "cx": cx,
                "cy": cy,
                "area": area,
                "robot_x": float(rx),
                "robot_y": float(ry),
                "bbox": [float(x), float(y), float(x + w), float(y + h)],
            }
            all_objs.append(obj)
            if show_debug:
                cv2.rectangle(debug_img, (x, y), (x + w, y + h), (255, 255, 0), 2)
                cv2.putText(
                    debug_img,
                    obj["label"],
                    (x, max(15, y - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                    1,
                )

    if show_debug:
        cv2.imshow("hybrid_hsv_debug", debug_img)
        cv2.waitKey(1)

    return all_objs


def detect_all_yolo(img, model, kb, conf_thres=0.4, show_debug=False):
    if model is None:
        return []

    all_objs = []
    debug_img = img.copy()
    results = model(img, verbose=False)
    if len(results) == 0:
        return []

    result = results[0]
    if result.boxes is None:
        return []

    names = result.names
    for box in result.boxes:
        conf = float(box.conf[0].item())
        if conf < conf_thres:
            continue

        cls_id = int(box.cls[0].item())
        cls_name = names[cls_id]
        label, object_name, color = normalize_yolo_class(cls_name)

        xyxy = box.xyxy[0].cpu().numpy()
        x1, y1, x2, y2 = [float(v) for v in xyxy]
        cx = float((x1 + x2) / 2.0)
        cy = float((y1 + y2) / 2.0)
        area = float(max(1.0, (x2 - x1) * (y2 - y1)))
        rx, ry = pixel_to_robot(cx, cy, kb)

        obj = {
            "source": "yolo",
            "class": cls_name,
            "label": label,
            "object_name": object_name,
            "color": color,
            "cx": cx,
            "cy": cy,
            "area": area,
            "conf": conf,
            "robot_x": float(rx),
            "robot_y": float(ry),
            "bbox": [x1, y1, x2, y2],
        }
        all_objs.append(obj)

        if show_debug:
            cv2.rectangle(debug_img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(
                debug_img,
                "%s:%.2f" % (label, conf),
                (int(x1), max(15, int(y1) - 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
            )

    if show_debug:
        cv2.imshow("hybrid_yolo_debug", debug_img)
        cv2.waitKey(1)

    return all_objs


def detect_all(img, hsv_cfg, kb, yolo_model=None, conf_thres=0.4, min_area=1500, show_debug=False):
    """
    YOLO-World + HSV 双重检验检测策略：
    1. 先用 YOLO-World 检测开放词汇物体（banana, cup, block 等）
    2. 对 YOLO 检测到的 block，在其 bbox 内用 HSV 进行颜色双重验证
    3. HSV 同时独立运行，补充检测 YOLO 漏检的 block（根据颜色）
    4. 对于 banana, cup 等非 block 物体，直接使用 YOLO 结果
    """
    # ========== 步骤 1: YOLO-World 检测 ==========
    yolo_objs = detect_all_yolo(img, yolo_model, kb, conf_thres=conf_thres, show_debug=show_debug)
    
    # ========== 步骤 2: HSV 基础检测（用于补充和验证） ==========
    # 始终运行 HSV，用于：1) 补充漏检 2) 验证颜色 3) 兜底
    hsv_objs = detect_all_hsv(img, hsv_cfg, kb, min_area=min_area, show_debug=False)
    
    # 如果 YOLO 完全没加载或没检测到任何东西，直接返回 HSV 结果（纯颜色模式）
    if not yolo_objs:
        rospy.loginfo("[Detect] YOLO-World 未检测到物体，使用 HSV 颜色检测模式")
        if show_debug and hsv_objs:
            # 显示HSV调试图像
            hsv_debug = img.copy()
            for obj in hsv_objs:
                x1, y1, x2, y2 = map(int, obj["bbox"])
                cv2.rectangle(hsv_debug, (x1, y1), (x2, y2), (255, 255, 0), 2)
                cv2.putText(hsv_debug, obj["label"], (x1, max(15, y1 - 5)),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            cv2.imshow("hsv_fallback_debug", hsv_debug)
            cv2.waitKey(1)
        
        # 为HSV结果添加ID
        for idx, obj in enumerate(hsv_objs):
            obj["id"] = idx
        return hsv_objs
    
    # ========== 步骤 3: 融合策略（双重检验） ==========
    final_objs = []
    used_hsv_indices = set()
    
    rospy.loginfo(f"[Detect] YOLO-World 检测到 {len(yolo_objs)} 个物体，开始进行 HSV 双重检验...")
    
    # 处理每个 YOLO 检测结果
    for obj in yolo_objs:
        # 情况 A: 检测到 block 类物体 → 需要 HSV 颜色双重验证
        if obj["object_name"] == "block":
            # 在该 bbox 区域内用 HSV 精确判断颜色
            hsv_color = classify_color_in_bbox(img, hsv_cfg, obj["bbox"])
            
            if hsv_color != "any":
                # 双重验证成功：使用 YOLO 的位置 + HSV 的颜色
                original_color = obj["color"]
                obj["color"] = hsv_color
                obj["label"] = f"{hsv_color}_block"
                obj["class"] = f"{hsv_color}_block"
                rospy.loginfo(f"[Detect] Block 双重检验: YOLO位置 + HSV颜色({original_color}→{hsv_color})")
            else:
                rospy.logwarn(f"[Detect] Block 颜色验证失败: YOLO检测到block但HSV无法确定颜色，保持YOLO原标签")
            
            # 查找是否有重合的 HSV 检测框（用于去重）
            best_iou = 0.0
            best_hsv_idx = None
            for idx, hsv_obj in enumerate(hsv_objs):
                if hsv_obj["object_name"] != "block":
                    continue
                iou = bbox_iou(obj["bbox"], hsv_obj["bbox"])
                if iou > best_iou:
                    best_iou = iou
                    best_hsv_idx = idx
            
            # 如果 IOU > 0.3，认为是同一个物体，标记该 HSV 为已使用
            if best_iou > 0.3 and best_hsv_idx is not None:
                used_hsv_indices.add(best_hsv_idx)
                rospy.loginfo(f"[Detect] YOLO block 与 HSV block 匹配成功(IOU={best_iou:.2f})，去重")
        
        # 情况 B: 检测到 banana, cup 等开放词汇物体 → 直接使用 YOLO 结果，无需 HSV 验证
        else:
            rospy.loginfo(f"[Detect] YOLO 检测到开放词汇物体: {obj['object_name']}({obj['label']})，置信度 {obj['conf']:.2f}")
        
        final_objs.append(obj)
    
    # ========== 步骤 4: HSV 补充检测（YOLO 漏检的 block） ==========
    for idx, hsv_obj in enumerate(hsv_objs):
        if idx in used_hsv_indices:
            continue  # 已匹配，跳过
        
        # 只补充 block 类物体（HSV 本来也只会检测 block）
        if hsv_obj["object_name"] == "block":
            final_objs.append(hsv_obj)
            rospy.loginfo(f"[Detect] HSV 补充检测到 YOLO 漏检的 {hsv_obj['color']}_block")
    
    # ========== 步骤 5: 添加 ID 并返回 ==========
    for idx, obj in enumerate(final_objs):
        obj["id"] = idx
    
    rospy.loginfo(f"[Detect] 最终检测结果: {len(final_objs)} 个物体 "
                  f"(YOLO:{len(yolo_objs)}个, HSV补充:{len(final_objs)-len(yolo_objs)}个)")
    
    return final_objs


# ============================
# 任务解析模块（LLM 支持）
# ============================

class LLMTaskParser:
    """
    大模型任务解析器
    调用 airbox 本地 Genie/OpenAI-compatible API 进行意图识别
    """
    def __init__(self, 
                 base_url="http://127.0.0.1:8910/v1",
                 model="DeepSeek-R1-Distill-Qwen-7B",
                 timeout=20.0,
                 max_retries=3):
        self.base_url = base_url
        self.model = model
        self.timeout = timeout
        self.max_retries = max_retries
        self.api_endpoint = f"{base_url}/chat/completions"
        
        # 系统提示词 - 严格约束输出格式，包含 object_name 字段
        self.system_prompt = """你是一个任务解析器。将用户输入的自然语言指令解析为JSON格式。
规则：
1. 必须只输出JSON，禁止任何解释、 markdown 代码块标记或额外文本；
2. color只能是any/red/green/blue；
3. position只能是any/left/right；
4. place_name只能是A/B/C/D；
5. object_name只能是block/banana/cup/mug/any；
6. 如果包含多个子任务，每个子任务必须是tasks数组中的独立对象；
7. raw字段必须保留原始子任务描述。

示例输入："把红色方块放到B区，然后绿色方块放到C区"
示例输出：{"tasks":[{"color":"red","object_name":"block","position":"any","place_name":"B","raw":"把红色方块放到B区"},{"color":"green","object_name":"block","position":"any","place_name":"C","raw":"绿色方块放到C区"}]}"""

        rospy.loginfo(f"[LLM] 初始化解析器: {self.api_endpoint}, 模型: {self.model}")

    def parse(self, text):
        """
        调用 LLM 解析任务文本
        返回: tasks 列表 或 None（失败时）
        """
        payload = {
            "model": self.model,
            "messages": [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": f"task_text: {text}"}
            ],
            "stream": False,
            "temperature": 0.0,
            "temp": 0.0,
            "top_k": 1,
            "top_p": 1.0
        }

        for attempt in range(self.max_retries):
            try:
                rospy.loginfo(f"[LLM] 第{attempt+1}次尝试解析: {text[:50]}...")
                
                response = requests.post(
                    self.api_endpoint,
                    headers={"Content-Type": "application/json"},
                    json=payload,
                    timeout=self.timeout
                )
                
                if response.status_code != 200:
                    rospy.logwarn(f"[LLM] HTTP错误: {response.status_code}, {response.text}")
                    continue
                
                result = response.json()
                content = result["choices"][0]["message"].get("content", "")
                
                # 清理可能的 markdown 代码块
                content = self._clean_json_content(content)
                
                # 解析 JSON
                parsed = json.loads(content)
                
                # 验证格式
                if self._validate_tasks(parsed):
                    rospy.loginfo(f"[LLM] 解析成功: {len(parsed['tasks'])} 个任务")
                    # 规范化每个任务
                    tasks = [normalize_task_rule(t) for t in parsed["tasks"]]
                    return tasks
                else:
                    rospy.logwarn(f"[LLM] 格式验证失败: {content}")
                    
            except requests.exceptions.Timeout:
                rospy.logwarn(f"[LLM] 请求超时 (>{self.timeout}s)")
            except requests.exceptions.ConnectionError:
                rospy.logwarn(f"[LLM] 连接错误: 无法连接到 {self.api_endpoint}")
            except json.JSONDecodeError as e:
                rospy.logwarn(f"[LLM] JSON解析错误: {e}, 原始内容: {content[:200]}")
            except Exception as e:
                rospy.logwarn(f"[LLM] 未知错误: {e}")
            
            if attempt < self.max_retries - 1:
                time.sleep(5)  # 重试前等待
        
        return None

    def _clean_json_content(self, content):
        """清理 LLM 输出中可能存在的 markdown 标记"""
        content = (content or "").strip()
        content = re.sub(r"<think>.*?</think>", "", content, flags=re.DOTALL).strip()
        content = re.sub(r"^```(?:json)?", "", content).strip()
        content = re.sub(r"```$", "", content).strip()

        start = content.find("{")
        end = content.rfind("}")
        if start != -1 and end != -1 and end > start:
            content = content[start:end + 1]

        return content.strip()

    def _validate_tasks(self, parsed):
        """验证返回的 JSON 结构是否符合预期"""
        if not isinstance(parsed, dict):
            return False
        if "tasks" not in parsed:
            return False
        if not isinstance(parsed["tasks"], list):
            return False
        
        required_fields = {"color", "object_name", "position", "place_name", "raw"}
        valid_colors = {"any", "red", "green", "blue"}
        valid_positions = {"any", "left", "right"}
        valid_places = {"A", "B", "C", "D"}
        valid_objects = {"any", "block", "banana", "cup", "mug"}
        
        for task in parsed["tasks"]:
            if not isinstance(task, dict):
                return False
            if not required_fields.issubset(set(task.keys())):
                return False
            if task["color"] not in valid_colors:
                return False
            if task["position"] not in valid_positions:
                return False
            if task["place_name"] not in valid_places:
                return False
            if task["object_name"] not in valid_objects:
                return False
        
        return True


def parse_task_legacy(text):
    """
    原有的本地正则解析函数（作为降级方案保留）
    """
    text = text.replace("然后", ";")
    parts = text.split(";")
    tasks = []
    for p in parts:
        color = "any"
        if "蓝" in p: color = "blue"
        if "红" in p: color = "red"
        if "绿" in p: color = "green"
        pos = "any"
        if "左" in p: pos = "left"
        if "右" in p: pos = "right"
        dest = "A"
        for d in ["A","B","C","D"]:
            if d in p: dest = d
        
        # 推断 object_name
        obj_name = "block"
        if "香蕉" in p or "banana" in p.lower():
            obj_name = "banana"
        elif "杯" in p or "cup" in p.lower() or "mug" in p.lower():
            obj_name = "cup"
        
        tasks.append({
            "color": color, 
            "object_name": obj_name,
            "position": pos, 
            "place_name": dest, 
            "raw": p
        })
    return tasks


def parse_task_with_fallback(text, llm_parser=None, prefer_llm=True):
    """
    智能解析函数：优先使用 LLM，失败时自动降级到本地解析
    """
    # 如果 prefer_llm=False 或没有提供 llm_parser，直接使用本地解析
    if not prefer_llm or llm_parser is None:
        rospy.loginfo("[Parse] 使用本地正则解析（离线模式）")
        return parse_task_legacy(text)
    
    # 尝试 LLM 解析
    llm_result = llm_parser.parse(text)
    if llm_result is not None:
        rospy.loginfo("[Parse] 使用 LLM 解析成功")
        return llm_result
    
    # LLM 失败，降级到本地
    rospy.logwarn("[Parse] LLM 解析失败，降级到本地正则解析")
    return parse_task_legacy(text)


def select_object(objs, rule):
    """根据规则选择物体"""
    cand = objs
    if rule["color"] != "any":
        cand = [o for o in cand if o["color"] == rule["color"]]
    if rule["object_name"] != "any":
        cand = [o for o in cand if o["object_name"] == rule["object_name"]]
    if not cand:
        return None
    if rule["position"] == "left":
        return min(cand, key=lambda x: x["cx"])
    if rule["position"] == "right":
        return max(cand, key=lambda x: x["cx"])
    return max(cand, key=lambda x: x["area"])


# ============================
# 状态机状态类
# ============================

class FindAll(smach.State):
    def __init__(self, yolo_model=None):
        super().__init__(outcomes=['success', 'timeout'], 
                         input_keys=['hsv','kb'], 
                         output_keys=['objects'])
        self.bridge = CvBridge()
        self.client = None
        self.yolo_model = yolo_model

    def move_to_search_pose(self):
        """先移动到搜索位姿，让摄像头向下看桌面"""
        if self.client is None:
            action_name = rospy.get_param("~arm_name", "sgr532") + "/sgr_ctrl"
            self.client = actionlib.SimpleActionClient(action_name, SGRCtrlAction)
            if not self.client.wait_for_server(timeout=rospy.Duration(5.0)):
                rospy.logerr("[DIAG] 无法连接机械臂控制服务")
                return False
        
        goal = SGRCtrlGoal()
        goal.action_type = goal.ACTION_TYPE_XYZ_RPY
        goal.grasp_type = goal.GRASP_OPEN
        goal.pos_x = 0.20
        goal.pos_y = 0.00
        goal.pos_z = 0.15
        goal.pos_roll = 0.0
        goal.pos_pitch = 1.57
        goal.pos_yaw = 0.0
        
        rospy.loginfo("[DIAG] 移动到搜索位姿...")
        self.client.send_goal_and_wait(goal, rospy.Duration(10.0))
        result = self.client.get_result()
        if result and result.result == SGRCtrlResult.SUCCESS:
            rospy.loginfo("[DIAG] 到达搜索位姿")
            rospy.sleep(0.5)
            return True
        return False

    def execute(self, ud):
        rospy.loginfo("[DIAG] >>> 进入 FindAll 状态")
        
        if not self.move_to_search_pose():
            rospy.logwarn("[DIAG] 无法移动到搜索位姿，但继续尝试识别")
        
        try:
            rospy.loginfo("[DIAG] 等待摄像头图像，超时5秒...")
            msg = rospy.wait_for_message("/usb_cam/image_raw", Image, timeout=5.0)
            rospy.loginfo("[DIAG] 成功获取图像")
            
            img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo(f"[DIAG] 转换图像成功，尺寸: {img.shape}")
            
            # 使用新的双重检验检测函数
            show_debug = rospy.get_param("~show_debug", False)
            conf_thres = rospy.get_param("~yolo_conf_thres", 0.4)
            min_area = rospy.get_param("~min_area", 1500)
            
            objs = detect_all(img, ud.hsv, ud.kb, 
                            yolo_model=self.yolo_model, 
                            conf_thres=conf_thres, 
                            min_area=min_area, 
                            show_debug=show_debug)
            
            ud.objects = objs
            rospy.loginfo(f"[DIAG] 检测到 {len(objs)} 个物体")
            
            if objs:
                for i, o in enumerate(objs):
                    rospy.loginfo(f"  物体 {i}: {o['color']} {o['object_name']} at ({o['robot_x']:.3f}, {o['robot_y']:.3f}), 来源: {o['source']}")
            
            return 'success'
        except rospy.ROSException as e:
            rospy.logerr(f"[DIAG] 摄像头获取失败: {e}")
            ud.objects = []
            return 'timeout'


class ParseTask(smach.State):
    """
    修改后的任务解析状态：只进行规则匹配，不再调用 LLM 解析文本
    文本解析在状态机启动前已完成，规则通过 userdata['task_rules'] 传入
    """
    def __init__(self):
        super().__init__(outcomes=['success'], 
                         input_keys=['objects', 'task_rules'],
                         output_keys=['tasks'])

    def execute(self, ud):
        rospy.loginfo("[DIAG] >>> 进入 ParseTask 状态（仅匹配物体，不解析文本）")
        
        # 直接使用预解析的规则，不再调用 LLM 或本地正则
        rules = ud.task_rules
        
        tasks = []
        for r in rules:
            obj = select_object(ud.objects, r)
            if obj is None:
                rospy.logwarn(f"[DIAG] 未匹配到物体: {r}")
                continue
            tasks.append({
                "pick": {"x": obj["robot_x"], "y": obj["robot_y"]},
                "place_name": r["place_name"],
                "reason": r["raw"]
            })
            rospy.loginfo(f"[DIAG] 匹配任务: {r['raw']} -> 物体 {obj['id']} ({obj['color']} {obj['object_name']})")
        
        ud.tasks = tasks
        rospy.loginfo(f"[DIAG] 共生成 {len(tasks)} 个任务")
        return 'success'


class ExecuteTask(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success','empty','invalid'],
                         input_keys=['tasks','objects'],
                         output_keys=['target','tasks','place'])

    def execute(self, ud):
        rospy.loginfo("[DIAG] >>> 进入 ExecuteTask 状态")
        if not ud.tasks:
            rospy.loginfo("[DIAG] 任务队列为空")
            return 'empty'

        t = ud.tasks.pop(0)
        rospy.loginfo(f"[DIAG] 当前任务: {t['reason']}, 期望位置: ({t['pick']['x']:.3f}, {t['pick']['y']:.3f})")
        
        obj, dist = find_nearest(ud.objects, t["pick"]["x"], t["pick"]["y"])
        rospy.loginfo(f"[DIAG] 最近物体: {obj['id'] if obj else None}, 距离: {dist:.4f}")

        if obj is None or dist > 0.03:
            rospy.logwarn("[DIAG] 目标匹配失败或距离过大")
            return 'invalid'

        ud.target = obj
        ud.place = t["place_name"]
        rospy.loginfo(f"[DIAG] 选中目标物体 {obj['id']} ({obj['color']} {obj['object_name']})，放置到 {t['place_name']}")
        return 'success'


class UpdatePlanningScene(smach.State):
    def __init__(self):
        super().__init__(
            outcomes=['success', 'error', 'invalid'],
            input_keys=['objects', 'target']
        )
        self.scene = None
        self.arm_group = None
        self.initialized = False
        self.ns = "sgr532"
        self.frame_id = None

    def execute(self, ud):
        rospy.loginfo("[DIAG] >>> 进入 UpdatePlanningScene 状态")
        
        if not hasattr(ud, "objects") or len(ud.objects) == 0:
            rospy.logwarn("[DIAG] object_list 为空")
            return 'invalid'
        if not hasattr(ud, "target"):
            rospy.logwarn("[DIAG] target 为空")
            return 'invalid'
            
        target_id = ud.target["id"]
        rospy.loginfo(f"[DIAG] 目标物体 ID: {target_id}, 总物体数: {len(ud.objects)}")

        if not self.initialized:
            try:
                rospy.loginfo(f"[DIAG] 初始化 PlanningScene (ns={self.ns})...")
                self.scene = moveit_commander.PlanningSceneInterface(ns=self.ns, synchronous=True)
                
                try:
                    rospy.loginfo("[DIAG] 尝试初始化 MoveGroupCommander...")
                    robot_desc = f"{self.ns}/robot_description"
                    self.arm_group = moveit_commander.MoveGroupCommander(
                        "sagittarius_arm",
                        robot_description=robot_desc,
                        ns=self.ns,
                        wait_for_servers=5.0
                    )
                    self.frame_id = self.arm_group.get_planning_frame()
                    rospy.loginfo(f"[DIAG] MoveGroupCommander 初始化成功，frame_id: {self.frame_id}")
                except Exception as e:
                    rospy.logwarn(f"[DIAG] MoveGroupCommander 初始化失败（可忽略）: {e}")
                    rospy.logwarn("[DIAG] 使用备用 frame_id: sgr532/base_link")
                    self.frame_id = "sgr532/base_link"
                
                self.initialized = True
                rospy.sleep(0.5)
                
            except Exception as e:
                rospy.logerr(f"[DIAG] PlanningScene 初始化失败: {e}")
                return 'success'

        try:
            rospy.loginfo("[DIAG] 清理旧障碍物...")
            self.scene.remove_world_object("table")
            for i in range(20):
                self.scene.remove_world_object(f"obs_{i}")
            rospy.sleep(0.3)

            rospy.loginfo(f"[DIAG] 添加桌面到 frame: {self.frame_id}")
            table_pose = PoseStamped()
            table_pose.header.frame_id = self.frame_id
            table_pose.pose.position.x = 0.25
            table_pose.pose.position.y = 0.00
            table_pose.pose.position.z = -0.01
            table_pose.pose.orientation.w = 1.0
            self.scene.add_box("table", table_pose, size=(0.50, 0.50, 0.02))

            added = 0
            for obj in ud.objects:
                if obj["id"] == target_id:
                    continue
                
                obs_pose = PoseStamped()
                obs_pose.header.frame_id = self.frame_id
                obs_pose.pose.position.x = obj["robot_x"]
                obs_pose.pose.position.y = obj["robot_y"]
                obs_pose.pose.position.z = 0.015
                obs_pose.pose.orientation.w = 1.0
                
                self.scene.add_box(
                    f"obs_{obj['id']}", 
                    obs_pose, 
                    size=(0.035, 0.035, 0.03)
                )
                added += 1
            
            rospy.sleep(0.5)
            rospy.loginfo(f"[DIAG] 场景更新完成，添加 {added} 个障碍物到 {self.frame_id}")
            return 'success'
            
        except Exception as e:
            rospy.logerr(f"[DIAG] 场景更新异常: {e}")
            return 'success'


class Grasp(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success','fail'], input_keys=['target'])
        self.client = None

    def execute(self, ud):
        rospy.loginfo("[DIAG] >>> 进入 Grasp 状态")
        
        if self.client is None:
            action_name = rospy.get_param("~arm_name", "sgr532") + "/sgr_ctrl"
            rospy.loginfo(f"[DIAG] 初始化 ActionClient: {action_name}")
            self.client = actionlib.SimpleActionClient(action_name, SGRCtrlAction)
            
            if not self.client.wait_for_server(timeout=rospy.Duration(10.0)):
                rospy.logerr(f"[DIAG] Action Server 连接超时！检查 sgr_ctrl_node")
                return 'fail'

        target = ud.target
        g = SGRCtrlGoal()
        g.action_type = g.ACTION_TYPE_PICK_XYZ
        g.pos_x = target["robot_x"]
        g.pos_y = target["robot_y"]
        g.pos_z = 0.01
        g.pos_pitch = 1.57

        rospy.loginfo(f"[DIAG] 发送抓取: ({g.pos_x:.3f}, {g.pos_y:.3f}, {g.pos_z:.3f}) - 目标: {target['color']} {target['object_name']}")
        
        self.client.send_goal_and_wait(g, rospy.Duration(30.0))
        result = self.client.get_result()
        
        if result and result.result == SGRCtrlResult.SUCCESS:
            rospy.loginfo("[DIAG] 抓取成功")
            return 'success'
        else:
            rospy.logwarn(f"[DIAG] 抓取失败，结果: {result}")
            return 'fail'


class Drop(smach.State):
    def __init__(self):
        super().__init__(outcomes=['success'], input_keys=['place'])
        self.client = None
        self.drop_dst = {
            "A": [0.15, -0.26, 0.2],
            "B": [0.15, 0.24, 0.2],
            "C": [0.26, -0.26, 0.2],
            "D": [0.26, 0.24, 0.2]
        }

    def execute(self, ud):
        rospy.loginfo("[DIAG] >>> 进入 Drop 状态")
        
        if self.client is None:
            action_name = rospy.get_param("~arm_name", "sgr532") + "/sgr_ctrl"
            self.client = actionlib.SimpleActionClient(action_name, SGRCtrlAction)
            if not self.client.wait_for_server(timeout=rospy.Duration(3.0)):
                rospy.logerr("[DIAG] Drop: Action Server 连接超时")
                return 'success'
        
        if ud.place not in self.drop_dst:
            rospy.logerr(f"[DIAG] 未知的放置区域: {ud.place}")
            return 'success'
            
        x, y, z = self.drop_dst[ud.place]
        g = SGRCtrlGoal()
        g.action_type = g.ACTION_TYPE_PUT_XYZ
        g.pos_x, g.pos_y, g.pos_z = x, y, z
        
        rospy.loginfo(f"[DIAG] 放置到 {ud.place}: ({x:.3f}, {y:.3f}, {z:.3f})")
        self.client.send_goal_and_wait(g)
        rospy.loginfo("[DIAG] 放置动作完成")
        return 'success'


# =============================
# 主程序
# =============================

class Demo:
    def get_task_text_from_param_or_voice(self):
        """
        获取任务文本。
        优先使用 launch 中的 ~task_text。
        如果 ~task_text 为空，则等待语音识别节点发布到 ~task_text_topic 的文本。
        如果等待超时或识别为空，则可回退到终端输入或 fallback_task_text。
        """
        task_text = rospy.get_param("~task_text", "").strip()

        if task_text:
            rospy.loginfo("[TaskInput] 使用 launch 中的 task_text: %s" % task_text)
            return task_text

        task_topic = rospy.get_param("~task_text_topic", "/voice_task_text")
        wait_timeout = float(rospy.get_param("~task_wait_timeout", 0.0))
        terminal_fallback = rospy.get_param("~terminal_fallback", True)
        fallback_task_text = rospy.get_param("~fallback_task_text", "把最左边的蓝色方块放到A区")

        rospy.loginfo("[TaskInput] task_text 为空，等待语音识别文本 topic: %s" % task_topic)

        try:
            if wait_timeout > 0:
                msg = rospy.wait_for_message(task_topic, String, timeout=wait_timeout)
            else:
                msg = rospy.wait_for_message(task_topic, String)

            text = (msg.data or "").strip()
            if text:
                rospy.loginfo("[TaskInput] 收到语音识别任务文本: %s" % text)
                return text
            else:
                rospy.logwarn("[TaskInput] 收到的语音识别文本为空")

        except rospy.ROSException as e:
            rospy.logwarn("[TaskInput] 等待语音识别文本失败: %s" % str(e))

        if terminal_fallback:
            try:
                text = input("语音识别文本为空或未收到，请手动输入任务：").strip()
                if text:
                    rospy.loginfo("[TaskInput] 使用终端输入任务文本: %s" % text)
                    return text
            except Exception as e:
                rospy.logwarn("[TaskInput] 终端输入失败: %s" % str(e))

        rospy.logwarn("[TaskInput] 使用 fallback_task_text: %s" % fallback_task_text)
        return fallback_task_text

    def __init__(self):
        # 视觉配置
        cfg = load_yaml(rospy.get_param("~vision_config"))
        self.kb = {
            "k1": cfg["LinearRegression"]["k1"],
            "b1": cfg["LinearRegression"]["b1"],
            "k2": cfg["LinearRegression"]["k2"],
            "b2": cfg["LinearRegression"]["b2"]
        }
        self.hsv = {
            c: {
                "lower": np.array([cfg[c]["hmin"]/2, cfg[c]["smin"], cfg[c]["vmin"]]),
                "upper": np.array([cfg[c]["hmax"]/2, cfg[c]["smax"], cfg[c]["vmax"]])
            } for c in ["red","green","blue"]
        }
        self.text = self.get_task_text_from_param_or_voice()
        
        # ========== 步骤 1: 先初始化 LLM 解析器 ==========
        self.use_llm = rospy.get_param("~use_llm", True)
        llm_url = rospy.get_param("~llm_url", "http://127.0.0.1:8910/v1")
        llm_model = rospy.get_param("~llm_model", "DeepSeek-R1-Distill-Qwen-7B")
        llm_timeout = rospy.get_param("~llm_timeout", 5.0)
        
        self.llm_parser = None
        if self.use_llm:
            try:
                self.llm_parser = LLMTaskParser(
                    base_url=llm_url,
                    model=llm_model,
                    timeout=llm_timeout,
                    max_retries=2
                )
                rospy.loginfo("[Demo] LLM 解析器初始化完成")
            except Exception as e:
                rospy.logerr(f"[Demo] LLM 解析器初始化失败: {e}")
                self.use_llm = False
        
        # ========== 步骤 2: 解析任务文本，得到 task_rules ==========
        rospy.loginfo(f"[Demo] 正在解析任务文本（LLM模式: {self.use_llm}）...")
        self.task_rules = parse_task_with_fallback(
            self.text,
            llm_parser=self.llm_parser,
            prefer_llm=self.use_llm
        )
        rospy.loginfo(f"[Demo] 任务解析完成，共 {len(self.task_rules)} 条规则: {self.task_rules}")
        
        # 从 task_rules 推断需要检测的物体类别
        self.yolo_classes = infer_object_classes_from_tasks(self.task_rules)
        rospy.loginfo(f"[Demo] 根据任务推断检测类别: {self.yolo_classes}")
        
        # ========== 步骤 3: 检查并自动下载 YOLO 模型 ==========
        self.yolo_model = None
        yolo_model_path = rospy.get_param("~yolo_model_path", "")
        auto_download = rospy.get_param("~auto_download_model", True)
        
        if yolo_model_path and YOLO is not None:
            # 检查模型文件是否存在
            if not os.path.exists(yolo_model_path):
                if auto_download:
                    rospy.logwarn(f"[Demo] 模型文件不存在: {yolo_model_path}")
                    rospy.loginfo("[Demo] 尝试自动下载模型...")
                    
                    # 尝试下载
                    success = download_yolo_model(yolo_model_path)
                    if not success:
                        rospy.logerr("[Demo] 模型下载失败，将仅使用 HSV 检测")
                        self.yolo_model = None
                else:
                    rospy.logerr(f"[Demo] 模型文件不存在且禁用自动下载: {yolo_model_path}")
                    self.yolo_model = None
            
            # 如果模型文件存在（或下载成功），则加载
            if os.path.exists(yolo_model_path):
                try:
                    rospy.loginfo(f"[Demo] 正在加载 YOLO-World 模型: {yolo_model_path}")
                    self.yolo_model = YOLO(yolo_model_path)
                    
                    # 如果是 YOLO-World 模型，动态设置类别
                    if "world" in yolo_model_path.lower():
                        rospy.loginfo(f"[Demo] 检测到 YOLO-World 模型，动态设置开放词汇类别: {self.yolo_classes}")
                        self.yolo_model.set_classes(self.yolo_classes)
                    else:
                        rospy.loginfo("[Demo] 使用标准 YOLO 模型，不设置开放词汇")
                    
                    rospy.loginfo("[Demo] YOLO-World 模型加载成功")
                except Exception as e:
                    rospy.logerr(f"[Demo] YOLO-World 模型加载失败: {e}")
                    self.yolo_model = None
        else:
            if not yolo_model_path:
                rospy.logwarn("[Demo] 未配置 YOLO 模型路径，将仅使用 HSV 检测")
            elif YOLO is None:
                rospy.logwarn("[Demo] ultralytics 未安装，将仅使用 HSV 检测")

    def run(self):
        sm = smach.StateMachine(outcomes=['done'])
        sm.userdata.kb = self.kb
        sm.userdata.hsv = self.hsv
        sm.userdata.text = self.text
        sm.userdata.task_rules = self.task_rules
        sm.userdata.tasks = []

        with sm:
            # 传入 yolo_model 到 FindAll 状态
            smach.StateMachine.add('FIND', FindAll(yolo_model=self.yolo_model),
                                   transitions={'success':'PARSE',
                                               'timeout':'FIND'})
            
            smach.StateMachine.add('PARSE', ParseTask(),
                                   transitions={'success':'EXEC'})
            
            smach.StateMachine.add('EXEC', ExecuteTask(),
                                   transitions={
                                       'success':'PLAN',
                                       'empty':'done',
                                       'invalid':'FIND'
                                   })
            smach.StateMachine.add('PLAN', UpdatePlanningScene(),
                                   transitions={
                                       'success': 'GRASP',
                                       'invalid': 'FIND',
                                       'error': 'FIND'
                                   })
            smach.StateMachine.add('GRASP', Grasp(),
                                   transitions={
                                       'success':'DROP',
                                       'fail':'FIND'
                                   })
            smach.StateMachine.add('DROP', Drop(),
                                   transitions={'success':'FIND'})

        rospy.loginfo("====================")
        rospy.loginfo("状态机启动，YOLO-World + HSV 双重检验模式")
        rospy.loginfo(f"动态检测类别: {self.yolo_classes}")
        if self.yolo_model:
            rospy.loginfo("YOLO-World 已启用")
        else:
            rospy.loginfo("仅使用 HSV 检测模式")
        rospy.loginfo("====================")
        sm.execute()


if __name__ == "__main__":
    rospy.init_node("llm_safe_sort")
    Demo().run()
