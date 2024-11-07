from flask import Flask, Response
import cv2

app = Flask(__name__)

# Inicialización de la captura de video desde la webcam
camera = cv2.VideoCapture(1)  # Asegúrate de que la webcam esté en el índice correcto

def generate_frames():
    while True:
        # Lee el cuadro de la cámara
        success, frame = camera.read()
        if not success:
            break
        else:
            # Codifica el cuadro en formato JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

            # Usa "multipart/x-mixed-replace" para transmitir los cuadros continuos
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    # La ruta entrega el flujo de video al cliente
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)  # Servidor expuesto en el puerto 5000
