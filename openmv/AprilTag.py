'''
此代码为AprilTag的检测代码，当检测到AprilTag时，会调用控制舵机的函数，使舵机做出相应的动作。
'''
import sensor,time, math
import duoji


class AprilTag():
    '''
    初始化摄像头，并设置像素格式，分辨率，确定图像的中心点
    '''
    def init(self):
        sensor.reset()  #该函数用于重置摄像头模块，确保其处于初始状态。
        sensor.set_pixformat(sensor.GRAYSCALE)  #该函数设置摄像头的像素格式为RGB565，这是一种常见的颜色编码格式，每个像素由16位表示，其中5位用于红色，6位用于绿色，5位用于蓝色。
        sensor.set_framesize(sensor.QVGA) # 该函数设置摄像头的分辨率为QQVGA（160x120像素），这是一个较低的分辨率，适合于快速处理和检测任务。
        sensor.skip_frames(30)  #该函数让摄像头跳过前30帧图像，以便摄像头能够自动调整曝光和白平衡等参数，确保后续捕获的图像质量更好。
        sensor.set_auto_gain(True)  # 该函数关闭摄像头的自动增益功能，防止图像过亮或过暗，保持图像质量稳定。
        sensor.set_auto_whitebal(True)  # 该函数关闭摄像头的自动白平衡功能，防止图像颜色失真，保持颜色稳定。
        self.clock = time.clock()  #该函数创建一个时钟对象，用于测量代码的执行时间和计算帧率（FPS）。通过调用clock.tick()可以更新时钟，并通过clock.fps()获取当前的帧率。

        # f_x 是x的像素为单位的焦距。对于标准的OpenMV，应该等于2.8/3.984*656，这个值是用毫米为单位的焦距除以x方向的感光元件的长度，乘以x方向的感光元件的像素（OV7725）
        # f_y 是y的像素为单位的焦距。对于标准的OpenMV，应该等于2.8/2.952*488，这个值是用毫米为单位的焦距除以y方向的感光元件的长度，乘以y方向的感光元件的像素（OV7725）

        # c_x 是图像的x中心位置
        # c_y 是图像的y中心位置

        self.f_x = (2.8 / 3.984) * 160 # 默认值
        self.f_y = (2.8 / 2.952) * 120 # 默认值
        self.c_x = 160 * 0.5 # 默认值(image.w * 0.5)
        self.c_y = 120 * 0.5 # 默认值(image.h * 0.5)

        self.DuoJi = duoji.Duoji() #创建一个舵机的对象，方便控制


    '''
    该函数将弧度转换为角度
    '''
    def degrees(self, radians):  #它接受一个弧度值作为输入，并返回对应的角度值。转换公式是：角度 = (180 * 弧度) / π，其中π是数学常数，约等于3.14159。通过调用这个函数，可以将摄像头检测到的旋转信息从弧度转换为更易理解的角度单位。
        return (180 * radians) / math.pi


    '''
    拍摄一张图片，然后框画出AprilTag,并标记出中心点，返回图像距离当前的位置信息，以及帧率
    '''
    def paishe(self):
        self.clock.tick()  #该函数更新时钟对象的计时器，通常在每次开始时调用，以便测量代码的执行时间和计算帧率（FPS）。通过调用clock.fps()可以获取当前的帧率。
        img = sensor.snapshot()  #该函数捕获一帧图像并返回一个图像对象。这个图像对象包含了摄像头捕获的图像数据，可以用于后续的图像处理和分析。在这个代码中，捕获的图像将被用于检测April Tags。

        # 初始化返回值（避免未检测到标签时报错）
        shuju = None  # 位置和旋转信息
        tag_id = None  # 检测到的标签ID
        biaoqian = False  # 标记是否检测到标签

        for tag in img.find_apriltags(fx=self.f_x, fy=self.f_y, cx=self.c_x, cy=self.c_y): # 默认为TAG36H11 tag36h11是AprilTag的一种类型，具有特定的编码和结构。通过调用img.find_apriltags()函数，可以在捕获的图像中检测到这种类型的April Tag，并返回一个包含检测结果的列表。每个检测结果包含了标签的位置、旋转信息等数据，可以用于后续的处理和分析。
            biaoqian = True #检测到标签了

            img.draw_rectangle(tag.rect, color = (255, 0, 0))  #该函数在图像上绘制一个矩形框，框住检测到的April Tag。tag.rect包含了矩形框的位置和大小信息，color参数指定了矩形框的颜色，这里使用红色（255, 0, 0）。通过调用这个函数，可以在图像上可视化地标记出检测到的April Tag的位置。
            img.draw_cross(tag.cx,tag.cy, color = (0, 255, 0)) #该函数在图像上绘制一个十字标记，标记出检测到的April Tag的中心位置。tag.cx和tag.cy分别表示标签中心的x和y坐标，color参数指定了十字标记的颜色，这里使用绿色（0, 255, 0）。通过调用这个函数，可以在图像上可视化地标记出检测到的April Tag的中心位置。

            #封装数据
            shuju = (
                tag.x_translation, tag.y_translation, tag.z_translation,
                self.degrees(tag.x_rotation), self.degrees(tag.y_rotation), self.degrees(tag.z_rotation))#该代码将检测到的April Tag的位置和旋转信息存储在一个元组print_args中。tag.x_translation、tag.y_translation和tag.z_translation分别表示标签在x、y和z轴上的平移位置，而tag.x_rotation、tag.y_rotation和tag.z_rotation分别表示标签绕x、y和z轴的旋转角度。通过调用self.degrees()函数，将旋转信息从弧度转换为角度，以便更易理解。最后，通过print()函数将这些信息输出到控制台，供用户查看。
            tag_id = tag.id

            # 位置的单位是未知的，旋转的单位是角度
            print("当前图片ID为：%d" % tag_id) #该代码使用print()函数输出当前检测到的April Tag的ID到控制台。tag.id表示标签的唯一标识符，通过调用这个代码，用户可以在控制台上查看检测到的April Tag的ID信息。
            #这里目标图片的居中条件为，Tx:0(左负右正) Ty:0(上正下负) Tz:-2.9(越远数越小，理论为0，但没必要靠太近) Rx:180（仰越小俯越大） Ry:0（向左偏时将从360递减（设最偏为320），右偏时将增大（设最偏为40）） Rz:0（顺时针翻转将增大，逆时针将从360递减）
            print("画面左右Tx: %.2f, 画面上下Ty: %.2f, 画面远近Tz: %.2f, 俯仰Rx: %.2f, 偏航Ry: %.2f, 翻滚Rz: %.2f" %(shuju[0],shuju[1],shuju[2],shuju[3],shuju[4],shuju[5]))
        fps=self.clock.fps()
        print("当前帧率: %.2f FPS" % fps)  #该代码使用print()函数输出当前的帧率（FPS）到控制台。通过调用clock.fps()函数，可以获取当前的帧率值，并将其打印出来。帧率表示每秒钟处理的图像帧数，较高的帧率通常意味着更流畅的图像处理和更快的响应速度。通过查看输出的帧率，用户可以评估代码的性能和效率.
        return shuju,tag_id,fps
    '''
    拍一张图片，并工具图片位置移动舵机做出相应的动作
    '''
    def run_kaishi(self,id):
        shuju,tag_id,fps = self.paishe() #先拍摄张图片，获取位置信息和帧率
        if fps > 6: #如果帧率大于6，则继续执行下面的代码，否则跳过，继续拍摄下一张图片
            if tag_id == id: # 如果检测到对应的图片，执行相应的舵机控制逻辑
                #如果图片合适则传输数据（xyz轴的坐标，以及RxRyRz轴的角度）到舵机
                if self.DuoJi.run_duoji(shuju[0],shuju[1],shuju[2],shuju[3],shuju[4],shuju[5]) == 0: # 调用舵机的控制函数，传入位置信息，控制舵机做出相应的动作。这里的位置信息单位是未知的，需要转换为舵机的角度信息。
                    return 0
            else:
                print("未检测到对应的图片，继续拍摄下一张图片") #输出提示信息，说明未检测到对应的图片，继续拍摄下一张图片
                return 1 #跳过下面的代码，继续拍摄下一张图片
        else:
            print("帧率过低，跳过当前帧") #输出提示信息，说明当前帧率过低，无法进行有效的处理。
            return 1#跳过下面的代码，继续拍摄下一张图片
