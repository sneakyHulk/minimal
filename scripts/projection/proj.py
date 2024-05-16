import cv2
import json
import numpy as np


def project_to_image(world_coordinates, projection_matrix):
    world_coordinates.append(1)
    tmp = np.matmul(projection_matrix, world_coordinates)  # np.array(projection_matrix) * np.array(world_coordinates)

    u = tmp[0] / abs(tmp[2]) if tmp[2] else tmp[0]
    v = tmp[1] / abs(tmp[2]) if tmp[2] else tmp[1]

    print(u, v)
    return [u, v]


def image_to_world(image_coordinates, projection_matrix, height):
    image_coordinates.append(1)

    KR = np.array(projection_matrix)[:, :3]
    KR_inv = np.linalg.inv(KR)

    tmp = np.matmul(KR_inv, image_coordinates)

    translation_camera = np.matmul(-KR_inv, np.array(np.array(projection_matrix)[:, -1]))

    tmp *= (height - translation_camera[2]) / tmp[2]
    tmp += translation_camera

    print(tmp[0], tmp[1], height)
    return [tmp[0], tmp[1], height]


def show_point(image_filename, world_coordinates, projection_matrix, real_image_coordinates=None):
    image_coordinates = project_to_image(world_coordinates, projection_matrix)

    image = cv2.imread(image_filename)

    image = cv2.circle(image, (int(image_coordinates[0]), int(image_coordinates[1])), radius=2, color=(0, 0, 255),
                       thickness=50)
    if real_image_coordinates:
        image = cv2.circle(image, (int(real_image_coordinates[0]), int(real_image_coordinates[1])), radius=2,
                           color=(0, 255, 0),
                           thickness=30)

    cv2.imshow("test", image)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    with open("2023-05-26_undistorted/projection_s110_n_cam_8.json", "rb") as file:
        camera_config = json.load(file)
        projection_matrix_s110_n_cam_8 = camera_config['projections'][0]['projection_matrix']

    with open("2023-05-26_undistorted/projection_s110_o_cam_8.json", "rb") as file:
        camera_config = json.load(file)
        projection_matrix_s110_o_cam_8 = camera_config['projections'][0]['projection_matrix']

    with open("2023-05-26_undistorted/projection_s110_s_cam_8.json", "rb") as file:
        camera_config = json.load(file)
        projection_matrix_s110_s_cam_8 = camera_config['projections'][0]['projection_matrix']

    with open("2023-05-26_undistorted/projection_s110_w_cam_8.json", "rb") as file:
        camera_config = json.load(file)
        projection_matrix_s110_w_cam_8 = camera_config['projections'][0]['projection_matrix']

    show_point(
        "/Users/lukas/src/2023-07-26/runs_videtec/run_3_undistorted/s110_s_cam_8/s110_s_cam_8_images/1690366190013.jpg",
        image_to_world([1505, 325], projection_matrix_s110_s_cam_8, 0), projection_matrix_s110_s_cam_8, [1505, 325])

    show_point(
        "/Users/lukas/src/2023-07-26/runs_videtec/run_3_undistorted/s110_s_cam_8/s110_s_cam_8_images/1690366190013.jpg",
        image_to_world([0, 1200], projection_matrix_s110_s_cam_8, 0), projection_matrix_s110_s_cam_8)

    show_point(
        "/Users/lukas/src/2023-07-26/runs_videtec/run_3_undistorted/s110_o_cam_8/s110_o_cam_8_images/1690366202851.jpg",
        image_to_world([0, 1200], projection_matrix_s110_o_cam_8, 0), projection_matrix_s110_o_cam_8)

    show_point(
        "/Users/lukas/src/2023-07-26/runs_videtec/run_3_undistorted/s110_n_cam_8/s110_n_cam_8_images/1690366190031.jpg",
        image_to_world([636.5, 229.5], projection_matrix_s110_n_cam_8, 0), projection_matrix_s110_n_cam_8, [636.5, 229.5])

    show_point(
        "/Users/lukas/src/2023-07-26/runs_videtec/run_3_undistorted/s110_n_cam_8/s110_n_cam_8_images/1690366190031.jpg",
        [0, 0, 1], projection_matrix_s110_n_cam_8)

    show_point(
        "/Users/lukas/src/2023-07-26/runs_videtec/run_3_undistorted/s110_w_cam_8/s110_w_cam_8_images/1690366190021.jpg",
        image_to_world([0, 1200], projection_matrix_s110_w_cam_8, 0), projection_matrix_s110_w_cam_8, [0, 1200])
