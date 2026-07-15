#本代码为maixcam通过网球识别模型标记唯一目标并计算误差
#首先需将对应的tennis模型的model文件和mud文件复制到maixcam内部model文件目录下方便调用
from maix import camera, display, image, nn, app, uart
import time

MODEL_PATH = "/root/models/tennis_1.mud" 

device = "/dev/ttyS0"
ser = uart.UART(device, 115200)

CENTER_X = 112
CENTER_Y = 112

detector = nn.YOLOv5(model=MODEL_PATH)

# 初始化摄像头
cam = camera.Camera(detector.input_width(), detector.input_height(), detector.input_format())
dis = display.Display()

print("Tennis Tracking Started...")

while not app.need_exit():
    img = cam.read()
    
    objs = detector.detect(img, conf_th=0.5, iou_th=0.45)
    
    best_obj = None
    max_score = 0
    
    for obj in objs:
        
        if obj.score > max_score:
            max_score = obj.score
            best_obj = obj

    if best_obj:
        img.draw_rect(best_obj.x, best_obj.y, best_obj.w, best_obj.h, color = image.COLOR_WHITE)

        cx = int(best_obj.x + best_obj.w / 2)
        cy = int(best_obj.y + best_obj.h / 2)
        
        err_x = cx - CENTER_X-7
        err_y = CENTER_Y - cy + 20
        
        img.draw_cross(cx, cy, image.COLOR_WHITE, 2)
        
        # Keep both err_x and err_y for a future two-axis gimbal. This board's
        # one-axis Error UART fast path currently receives horizontal err_x.
        data = f"{err_x}\n"
        ser.write(data.encode())
        
        msg = f"Score:{best_obj.score:.2f} Err:{err_x},{err_y},{1}"
        img.draw_string(0, 0, msg, color = image.COLOR_GREEN)
        
    else:
        msg = f"No Ball,Err:10,0,0"
        img.draw_string(0, 0, msg, color = image.COLOR_BLACK)
        data = "0\n"
        ser.write(data.encode())

    dis.show(img)
