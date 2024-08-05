import numpy as np
import cv2 as cv
import face_recognition as fr
import os


while True:
    print("""
    1 - Take picture
    2 - Show webcam
    0 - Quit
    """)
    
    option = int(input("Choose an option: "))

    match option:
        case 1:
            cap = cv.VideoCapture(0)

            name = input("Please write your name: ")
            if not cap.isOpened():
                print("Cannot open camera")
                exit()
            while True:
                ret, frame = cap.read()
                cv.imshow('frame', frame)
                wait_key = cv.waitKey(1)
                if wait_key == ord('p'):
                    cv.imwrite(f'arceus_recognition/src/images/{name}.png', cv.cvtColor(frame, cv.COLOR_BGR2RGB))
                elif wait_key == ord('q'):
                    break
            cv.destroyAllWindows()
            cap.release()

        case 2:
            cap = cv.VideoCapture(0)

            if not cap.isOpened():
                print("Cannot open camera")
                exit()

            process_this_frame = True
            known_face_encodings = []
            known_face_names = []

            for i in os.scandir('arceus_recognition/src/images'):
                image = fr.load_image_file(f"arceus_recognition/src/images/{i.name}")
                known_face_encodings.append(fr.face_encodings(image)[0])
                known_face_names.append(i.name.replace(".png", ""))


            while True:
                ret, frame = cap.read()
                
                if ret == True:
                    if process_this_frame:
                        small_frame = cv.resize(frame, (0, 0), fx=0.25, fy=0.25)
                        rgb_cam = cv.cvtColor(small_frame, cv.COLOR_BGR2RGB)
                        face_locations = fr.face_locations(rgb_cam)
                        face_encodings = fr.face_encodings(rgb_cam, face_locations)
                        face_names = []
                        
                        for face_encoding in face_encodings:
                            # See if the face is a match for the known face(s)
                            matches = fr.compare_faces(known_face_encodings, face_encoding)
                            name = "Unknown"

                            face_distances = fr.face_distance(known_face_encodings, face_encoding)
                            best_match_index = np.argmin(face_distances)
                            if matches[best_match_index]:
                                name = known_face_names[best_match_index]

                            face_names.append(name)

                    process_this_frame = not process_this_frame

                    for (top, right, bottom, left), name in zip(face_locations, face_names):
                        # Scale back up face locations since the frame we detected in was scaled to 1/4 size
                        top *= 4
                        right *= 4
                        bottom *= 4
                        left *= 4

                        # Draw a box around the face
                        cv.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

                        # Draw a label with a name below the face
                        cv.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv.FILLED)
                        font = cv.FONT_HERSHEY_DUPLEX
                        cv.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

                    # Display the resulting image
                    cv.imshow('Video', frame)

                    # Hit 'q' on the keyboard to quit!
                    if cv.waitKey(1) & 0xFF == ord('q'):
                        break
                        
                else:
                    print("ret error")
                    break

            cv.destroyAllWindows()    
            cap.release()

        case 0:
            cv.destroyAllWindows()
            cap.release()
            break
