import cv2
import numpy as np

# Load the image
img = cv2.imread("p11.jpg", 1)  # Read the image in color

# Convert the image to grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# Apply Gaussian blur to reduce noise
gray_blurred = cv2.GaussianBlur(gray, (5, 5), 0)

# Apply adaptive thresholding
thresh = cv2.adaptiveThreshold(gray_blurred, 255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C , cv2.THRESH_BINARY_INV, 11, 2)

# Find contours of the thresholded image
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# Filter and draw contours based on area or length threshold
min_contour_length = 50  # Minimum length threshold for a crack
longest_contour = None
longest_contour_length = 0
for contour in contours:
    contour_length = cv2.arcLength(contour, True)
    if contour_length > min_contour_length and contour_length > longest_contour_length:
        longest_contour = contour
        longest_contour_length = contour_length

# Draw the longest contour (crack) on the original image
if longest_contour is not None:
    cv2.drawContours(img, [longest_contour], -1, (0, 255, 0), 2)
    # Get the bounding rectangle of the longest contour
    x, y, w, h = cv2.boundingRect(longest_contour)
    # Display the length of the longest crack on the image
    cv2.putText(img, f"Length: {longest_contour_length:.2f}px", (x, y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Display the original image with the longest crack and its length
cv2.imshow("Crack Detection", img)
cv2.waitKey(3000)
cv2.imshow("tt Detection", thresh)
cv2.waitKey(0)
cv2.destroyAllWindows()
