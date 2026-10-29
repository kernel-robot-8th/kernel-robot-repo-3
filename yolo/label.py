from pathlib import Path  # 이미지 경로를 찾고 라벨 파일을 저장하는 도구를 가져온다.

import cv2  # 사진을 읽고 마우스로 박스를 선택하는 OpenCV를 가져온다.


class_id = 2  # 클래스 번호를 정한다. 딸기는 0, 자동차는 1, 공은 2이다.
class_name = 'ball'  # 이번에 라벨링할 클래스 이름을 정한다.
image_dir = (  # 실행 위치와 관계없이 원본 사진 폴더를 찾는다.
    Path(__file__).resolve().parents[1]  # 이 파일을 기준으로 프로젝트 폴더를 찾는다.
    / 'video'  # 원본 영상과 사진이 있는 폴더를 지정한다.
    / 'ball_frames'  # 공 사진이 있는 폴더를 지정한다.
)
images = sorted(image_dir.glob('*.jpg'))  # 현재 폴더에 있는 공 사진 전체를 이름순으로 가져온다.
if not images:  # 경로가 잘못되었거나 사진이 없는지 확인한다.
    raise SystemExit('이미지 폴더를 확인하세요.')  # 사진이 없으면 이유를 알리고 종료한다.

print(f'라벨링 대상: {len(images)}장 / 클래스: {class_id} = {class_name}')  # 작업 대상을 안내한다.
print('드래그 후 Enter: 저장 / 박스 없이 Enter: 이번 실행 종료')  # 조작 방법을 안내한다.

for index, image_path in enumerate(images, start=1):  # 사진을 첫 장부터 한 장씩 처리한다.
    label_path = image_path.with_suffix('.txt')  # 사진과 이름이 같은 txt 경로를 만든다.
    if label_path.exists():  # 이미 작업한 라벨이 있는지 확인한다.
        print(f'[{index}/{len(images)}] 기존 라벨 유지: {label_path.name}')  # 재실행 시 이어간다.
        continue  # 기존 라벨을 덮어쓰지 않고 다음 사진으로 이동한다.

    image = cv2.imread(str(image_path))  # 현재 원본 사진을 읽는다.
    if image is None:  # 사진이 손상되어 읽지 못했는지 확인한다.
        raise SystemExit(f'사진을 읽지 못했습니다: {image_path}')  # 실패한 파일을 알리고 종료한다.

    image_h, image_w = image.shape[:2]  # 원본 사진의 높이와 너비를 구한다.
    scale = min(1.0, 900 / image_h, 1200 / image_w)  # 큰 사진을 화면 안에 표시할 비율을 정한다.
    preview = cv2.resize(image, (round(image_w * scale), round(image_h * scale)))  # 표시용으로 축소한다.
    preview_h, preview_w = preview.shape[:2]  # 실제 표시용 사진의 높이와 너비를 구한다.
    title = f'{class_name} {index}/{len(images)} - {image_path.name}'  # 클래스와 진행 상황을 표시한다.
    print(f'[{index}/{len(images)}] 공 전체를 선택하세요: {image_path.name}')  # 현재 작업을 안내한다.
    x, y, w, h = cv2.selectROI(title, preview, showCrosshair=True)  # 드래그한 박스 좌표를 받는다.
    cv2.destroyAllWindows()  # 현재 사진 창을 닫는다.

    if w == 0 or h == 0:  # 선택 취소나 빈 박스 확정을 확인한다.
        print('종료합니다. 다시 실행하면 저장하지 않은 사진부터 이어갑니다.')  # 재실행을 안내한다.
        break  # 빈 라벨을 저장하지 않고 반복을 끝낸다.

    center_x = (x + w / 2) / preview_w  # 박스 중심 x를 표시용 사진 너비로 나눈다.
    center_y = (y + h / 2) / preview_h  # 박스 중심 y를 표시용 사진 높이로 나눈다.
    box_w = w / preview_w  # 박스 너비를 사진 전체 너비에 대한 비율로 바꾼다.
    box_h = h / preview_h  # 박스 높이를 사진 전체 높이에 대한 비율로 바꾼다.
    # 사진과 박스를 함께 축소했으므로 이 비율은 원본 사진에도 그대로 적용된다.
    label = f'{class_id} {center_x:.6f} {center_y:.6f} {box_w:.6f} {box_h:.6f}\n'  # 정답 한 줄을 만든다.
    label_path.write_text(label, encoding='utf-8')  # 클래스와 비율 좌표를 txt 파일로 저장한다.
    print(f'저장 완료: {label_path.name}')  # 저장 결과를 알리고 다음 사진으로 넘어간다.

completed = sum(path.with_suffix('.txt').exists() for path in images)  # 저장된 라벨 개수를 센다.
print(f'라벨 파일: {completed}/{len(images)}장')  # 현재까지의 작업 진행 상황을 출력한다.
