import cv2
from ultralytics import YOLO

# 1. Wczytaj swój autorski model
model = YOLO('best.pt') 

# 2. Otwórz kamerę (0 to zazwyczaj wbudowana kamera w laptopie, 1 to kamera USB)
cap = cv2.VideoCapture(0)

print("Kamera uruchomiona. Pokaż młotek lub butelkę! (Wciśnij 'q' aby wyjść)")

while cap.isOpened():
    success, frame = cap.read()
    if not success:
        print("Nie można odczytać obrazu z kamery.")
        break

    # 3. Przepuść klatkę z kamery przez sieć neuronową
    results = model(frame, conf=0.80)
    
    # 4. Narysuj ramki i etykiety na obrazie
    annotated_frame = results[0].plot()
    
    # 5. Wyświetl okienko z podglądem na żywo
    cv2.imshow("Wzrok Lazika - Test", annotated_frame)
    
    # Wciśnij 'q' na klawiaturze, aby zamknąć program
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Sprzątanie po zamknięciu
cap.release()
cv2.destroyAllWindows()