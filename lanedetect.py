import cv2 as cv
import numpy as np

def make_points(img, line):
    slope, intercept = line
    y1 = int(img.shape[0])
    y2 = int(y1*3/5)
    x1 = int((y1 - intercept)/slope)
    x2 = int((y2 - intercept)/slope)
    return [[x1, y1, x2, y2]]

def average_slope_intercept(img, lines):
    left_fit = []
    right_fit = []
    for line in lines:
        for x1, y1, x2, y2 in line:
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                left_fit.append((slope, intercept))
            else:
                right_fit.append((slope, intercept))
            
    left_fit_average = np.average(left_fit, axis=0)
    right_fit_average = np.average(right_fit, axis=0)
    left_line = make_points(img, left_fit_average)
    right_line = make_points(img, right_fit_average)
    averaged_lines = [left_line, right_line]
    return averaged_lines

def display_line(image, lines):
    line_image = np.zeros_like(image)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv.line(line_image, (x1, y1), (x2, y2), (255, 0, 0), 10)
    return line_image


if __name__ == "__main__":
    cap = cv.VideoCapture('test.mp4')
    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret == True:
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            edges = cv.Canny(gray, 50, 150, apertureSize=3)
            lanes_list = []
            lanes = cv.HoughLinesP(edges, 1, np.pi/180, 10, minLineLength=10, maxLineGap=10)
            averaged_lines = average_slope_intercept(frame, lanes)
            display_lines = display_line(frame, averaged_lines)
            
            cv.imshow('display_lines ', display_lines )
            cv.imshow('edges', edges)
            if cv.waitKey(25) & 0xFF == ord('q'):
                break
        else:
            break
    cap.release()
    cv.destroyAllWindows()