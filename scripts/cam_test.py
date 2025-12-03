import cv2

def main():
    cap = cv2.VideoCapture(0)  # 内蔵/一台目のWebカメラ

    if not cap.isOpened():
        print("カメラが開けませんでした")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("フレーム取得に失敗しました")
            break

        cv2.imshow("Webcam (press q to quit)", frame)

        # q を押したら終了
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
