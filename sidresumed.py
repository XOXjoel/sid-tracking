# Agregamos librerias necesarias
import cv2
import depthai as dai
import time
import blobconverter
import serial

# Empezamos comunicacion con arduino
arduino = serial.Serial('COM7', 9600)
time.sleep(2)

# Definimos el tamaño del frame
FRAME_SIZE = (640, 400)

# Definimos el modelo de la red neuronal, el nombre y el nombre del zoo
DET_INPUT_SIZE = (300, 300)
model_name = "face-detection-retail-0004"
zoo_type = "depthai"
blob_path = None

# Definimos un pipeline
pipeline = dai.Pipeline()

# Creamos un nodo para la camara rgb
cam = pipeline.createColorCamera()
# Le damos caracteristicas
cam.setPreviewSize(FRAME_SIZE[0], FRAME_SIZE[1])
cam.setInterleaved(False)
cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
cam.setBoardSocket(dai.CameraBoardSocket.RGB)

# Creamos nodo para la camara izquierda y derecha
mono_left = pipeline.createMonoCamera()
mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
mono_right = pipeline.createMonoCamera()
mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# Creamos nodo para la vision stereo (profundidad)
stereo = pipeline.createStereoDepth()
stereo.setLeftRightCheck(True)

# Asociamos las camaras mono al nodo stereo
mono_left.out.link(stereo.left)
mono_right.out.link(stereo.right)

# Usamos blobconverter para obtener el blob del modelo requerido
if model_name is not None:
    blob_path = blobconverter.from_zoo(
        name=model_name,
        shaves=6,
        zoo_type=zoo_type
    )

# Creamos nodo para la red neuronal de deteccion de caras
face_spac_det_nn = pipeline.createMobileNetSpatialDetectionNetwork()
# Le damos caracteristicas
face_spac_det_nn.setConfidenceThreshold(0.75)
face_spac_det_nn.setBlobPath(blob_path)
face_spac_det_nn.setDepthLowerThreshold(100)
face_spac_det_nn.setDepthUpperThreshold(5000)

# Creamos nodo para input de la configuracion de la deteccion de caras (manipulacion de imagen)
face_det_manip = pipeline.createImageManip()
# Le damos caracteristicas
face_det_manip.initialConfig.setResize(DET_INPUT_SIZE[0], DET_INPUT_SIZE[1])
face_det_manip.initialConfig.setKeepAspectRatio(False)

# Asociar
# Output RGB_Cam a Nodo de manipulacion de imagenes
# Output Nodo de manipulacion de imagenes a Input de la red neuronal
# Output de vision stereo (profundidad) a Nodo de la red neuronal
cam.preview.link(face_det_manip.inputImage)
face_det_manip.out.link(face_spac_det_nn.input)
stereo.depth.link(face_spac_det_nn.inputDepth)

# Creamos un nodo para una vista anticipada del output
x_preview_out = pipeline.createXLinkOut()
x_preview_out.setStreamName("preview")
cam.preview.link(x_preview_out.input)

# Creamos nodo para el video de la deteccion de la red neuronal
det_out = pipeline.createXLinkOut()
det_out.setStreamName('det_out')
face_spac_det_nn.out.link(det_out.input)

# Creamos un nodo para una vista anticipada del output
disparity_out = pipeline.createXLinkOut()
disparity_out.setStreamName("disparity")
stereo.disparity.link(disparity_out.input)


# Definimos una funcion para desplegar la informacion del frame de la imagen
def display_info(frame, disp_frame, bbox, coordinates, status, status_color):
    # Desplegamos la bounding box
    cv2.rectangle(frame, bbox, status_color[status], 2)

    # Desplegamos coordenadas
    if coordinates is not None:
        coord_x, coord_y, coord_z = coordinates
        cv2.putText(frame, f"X: {int(coord_x)} mm", (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.putText(frame, f"Y: {int(coord_y)} mm", (bbox[0] + 10, bbox[1] + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
        cv2.putText(frame, f"Z: {int(coord_z)} mm", (bbox[0] + 10, bbox[1] + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

    # Desplegamos el estatus de autenticidad en el frame
    cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color[status])

    # Desplegamos la bounding box y el valor de la profundidad en el frame de disparidad
    if coordinates is not None:
        cv2.rectangle(disp_frame, bbox, status_color[status], 2)
        cv2.rectangle(disp_frame, (5, 5, 185, 50), (50, 0, 0), -1)
        _, _, coord_z = coordinates
        cv2.putText(disp_frame, f'Depth: {coord_z}mm', (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255))

status_color = {
    'Face Detected': (255, 0, 255),
    'No Face Detected': (0, 0, 255)
}

# Empezar pipeline
with dai.Device(pipeline) as device:

    # Output queue will be used to get the right camera frames from the outputs defined above
    # El output queue se utilizará para obtener los frames de la cámara derecha de las salidas definidas anteriormente
    q_cam = device.getOutputQueue(name="preview", maxSize=1, blocking=False)

    # Output queue will be used to get nn data from the video frames.
    # El output queue se utilizará para obtener los datos de los frames del video de la red neuronal
    q_det = device.getOutputQueue(name="det_out", maxSize=1, blocking=False)

    # Output queue will be used to get disparity map from stereo node.
    # El output queue se utilizará para obtener el mapa de dispridad del nodo stereo
    q_disp = device.getOutputQueue(name="disparity", maxSize=1, blocking=False)


    while True:
        # Obtener frame de la camara derecha
        in_cam = q_cam.get() # retrieve data
        frame = in_cam.getCvFrame()

        # Obtener el frame de disparidad
        in_disp = q_disp.get()
        disp_frame = in_disp.getCvFrame() # hasta aqui entiendo

        # Calculate a multiplier for color mapping disparity map
        disparityMultiplier = 255 / stereo.getMaxDisparity()

        # Colormap disparity for display.
        disp_frame = (disp_frame * disparityMultiplier).astype('uint8')

        # Apply color map to disparity map
        disp_frame = cv2.applyColorMap(disp_frame, cv2.COLORMAP_JET)

        bbox = None
        coordinates = None

        inDet = q_det.tryGet()

        if inDet is not None:
            detections = inDet.detections

            # Condicion de una cara detectada
            if len(detections) is not 0:
                detection = detections[0]

                # Bounding box
                xmin = max(0, detection.xmin)
                ymin = max(0, detection.ymin)
                xmax = min(detection.xmax, 1)
                ymax = min(detection.ymax, 1)

                # Calcular coordenadas
                x = int(xmin*FRAME_SIZE[0])
                y = int(ymin*FRAME_SIZE[1])
                w = int(xmax*FRAME_SIZE[0]-xmin*FRAME_SIZE[0])
                h = int(ymax*FRAME_SIZE[1]-ymin*FRAME_SIZE[1])

                bbox = (x, y, w, h)

                # Coordenadas espaciales
                coord_x = detection.spatialCoordinates.x
                coord_y = detection.spatialCoordinates.y
                coord_z = detection.spatialCoordinates.z

                coordinates = (coord_x, coord_y, coord_z)

        # Checar si una cara fue detectada
        if bbox:
            status = 'Face Detected'

            if coord_z > 500 and coord_z < 2500:

                if coord_x < 100 and coord_x > -100:
                    #print("Go straight")
                    arduino.write(b'b')

                elif coord_x > 100:
                    #print("Turn right")
                    arduino.write(b'e')

                elif coord_x < -100:
                    #print("Turn left")
                    arduino.write(b'm')

            elif coord_z < 500:
                arduino.write(b'n')

                if coord_y > 30:
                    arduino.write(b'u')
                    print("arriba")

                elif coord_y < -30:
                    arduino.write(b'g')
                    print("abajo")

                elif coord_y > -30 and coord_y < 30:
                    arduino.write(b'z')
                    print("centro")


        else:
            status = 'No Face Detected' 
            arduino.write(b'c')

        # Display info on frame
        display_info(frame, disp_frame, bbox, coordinates, status, status_color)

        # Para el programa con
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # Desplega el frame final
        cv2.imshow("Face Cam", frame)

        # Desplega el frame de disparidad
        cv2.imshow("Disparity Map", disp_frame)

cv2.destroyAllWindows()