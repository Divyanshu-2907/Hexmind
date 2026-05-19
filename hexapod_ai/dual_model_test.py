import cv2
from ultralytics import YOLO

def main():
    print("Loading models... This may take a few seconds.")
    # Load both models into memory
    model_base = YOLO("models/yolov8n.pt")
    model_custom = YOLO("models/best.pt")
    
    cap = cv2.VideoCapture(0) # Open laptop webcam
    print("Running both models. Press 'q' in the video window to quit.")
    
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
            
        # Run predictions for both at the same time
        res_base = model_base.predict(frame, conf=0.35, verbose=False)[0]
        res_custom = model_custom.predict(frame, conf=0.35, verbose=False)[0]
        
        # Overlay both on the SAME frame
        # First draw base model predictions
        annotated = res_base.plot()
        
        # Then draw custom model predictions on top of the base annotations
        annotated = res_custom.plot(img=annotated)
        
        cv2.putText(annotated, "Dual YOLO: Official (Base) + Custom (best.pt)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        cv2.imshow("Single Frame - Dual YOLO Comparison", annotated)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
