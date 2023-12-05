import cv2
import numpy as np
def unflatten_index(flat_index, img_shape):
    row_size, col_size = img_shape[:2]
    row_index = flat_index // col_size
    col_index = flat_index % col_size
    return row_index, col_index



def plot_circle(img, index, radius=10, color=(0, 0, 255), thickness=2):
    row, col = unflatten_index(index, img.shape)
    img[row, col] = [255, 0, 0]
    # cv2.circle(img, (col, row), radius, color, thickness)
    return img
def segment_from_indexes(indexes):
    img = cv2.imread('img.jpeg')
    for index in indexes:
        img = plot_circle(img, index)
    #save img
    cv2.imwrite('segmented_img.jpeg', img)
# read img 
# img = cv2.imread('img.jpeg')
# no_of_pxls = img.shape[0]*img.shape[1]
# print('no_of_pxls:',no_of_pxls)
# # show img
# print('img_shape:',img.shape)
# img = plot_circle(img, 26314)
# cv2.imshow('img',img)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
