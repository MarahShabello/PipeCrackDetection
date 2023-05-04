import cv2
import numpy as np

# Load the image
img = cv2.imread("00002.jpg", 0)  # Read the image in grayscale

# Apply Gaussian blur to reduce noise
img_blurred = cv2.GaussianBlur(img, (5, 5), 0)

# Apply Canny edge detection
edges = cv2.Canny(img_blurred, 50, 150)  # Adjust the threshold values as needed

# Find contours of the edges
contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Calculate the length of the longest contour
longest_contour_length = 0
for contour in contours:
    contour_length = cv2.arcLength(contour, True)
    if contour_length > longest_contour_length:
        longest_contour_length = contour_length

# Display the original image and the detected edges
cv2.imshow("Original Image", img)
cv2.imshow("Canny Edges", edges)

# Display the length of the longest contour in pixels
print(f"Length of the longest crack in pixels: {longest_contour_length}")

cv2.waitKey(0)
cv2.destroyAllWindows()
