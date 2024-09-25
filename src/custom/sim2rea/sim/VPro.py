import cv2
import sys
import numpy as np
import os
import re


# 图片所在文件夹路径
images_folder = sys.path[0]+ '/VCap'  # 请替换为你的图像文件夹路径

# 获取文件夹中所有图像文件的路径
image_files = sorted([os.path.join(images_folder, file) for file in os.listdir(images_folder)],
                     key=lambda x: int(re.search(r'Img_(\d+)\.png', x).group(1)))

# 创建一个空白图像用于合成
composite_image = np.zeros_like(cv2.imread(image_files[0]))

# 循环遍历每张图片
for image_file in image_files:
    # 读取当前图片
    current_image = cv2.imread(image_file)

    # 在合成图像上绘制当前帧
    composite_image = cv2.addWeighted(composite_image, 1, current_image, 0.5, 0)

    # 显示当前合成图像（可选）
    cv2.imshow('Composite Image', composite_image)
    cv2.waitKey(30)  # 设置适当的等待时间

# # 保存合成图像
# cv2.imwrite('output_image.jpg', composite_image)

# # 关闭窗口
# cv2.destroyAllWindows()

