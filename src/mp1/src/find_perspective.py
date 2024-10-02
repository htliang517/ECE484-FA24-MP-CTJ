import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import cv2
import numpy as np

img_path = "./test_snow.png"
img = mpimg.imread(img_path)

plt.subplot(1, 2, 1)
plt.imshow(img)
plt.title("Original Image")
plt.axis('off')

def create_birds_eye_view(image):
    rows, cols, ch = image.shape
    print(cols, rows)
    # Define points for perspective transformation (example)
    # rows: y, cols: x
    # UL(x,y) - UR - LL - LR

    # From img.png
    # pts1 = np.float32([[490, 217], [605, 217], [382, rows], [881, rows]])
    # From test.png cols640, rows480
    # pts1 = np.float32([[282, 268], [360, 268], [75, 409], [554, 409]])
    # snow 1242x374
    pts1 = np.float32([[600, 280], [780, 280], [430, 368], [930, 368]])

    pts2 = np.float32([[0, 0], [cols, 0], [0, rows], [cols, rows]])
    
    M = cv2.getPerspectiveTransform(pts1, pts2)
    
    dst = cv2.warpPerspective(image, M, (cols, rows))
    return dst

img_bird_view = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
bird_eye_img = create_birds_eye_view(img_bird_view)

bird_eye_img_rgb = cv2.cvtColor(bird_eye_img, cv2.COLOR_BGR2RGB)

plt.subplot(1, 2, 2)
plt.imshow(bird_eye_img_rgb)
plt.title("Bird's-eye View")
plt.axis('off')

plt.show()
