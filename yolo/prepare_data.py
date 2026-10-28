from pathlib import Path  # 파일과 폴더 경로를 다루는 도구를 가져온다.
import shutil  # 원본을 유지하면서 파일을 복사하는 도구를 가져온다.


root = Path(__file__).resolve().parents[1]  # 프로젝트의 최상위 폴더를 찾는다.
dataset = root / 'yolo' / 'dataset_strawberry_car_ball'  # 세 클래스를 함께 학습할 데이터 폴더다.
sources = [  # 원본 폴더와 정답 클래스 번호를 연결한다.
    ('video/KakaoTalk_20261022_201351859_frames', 0),  # 딸기 사진이다.
    ('video/KakaoTalk_20261022_201218532_frames', 1),  # 자동차 사진이다.
    ('video/ball_frames', 2),  # 공 사진이다.
    ('yolo/camera_frames', None),  # camera_label.py로 찍은 사진이다. 여러 클래스가 섞여 있다.
]  # 폴더별로 학습과 검증을 따로 나눈다. None은 여러 클래스가 섞인 폴더다.
class_ids = {'0', '1', '2'}  # 라벨에 허용하는 클래스 번호다. 아래 data.yaml의 names와 같게 둔다.
pairs = []  # 검증을 통과한 사진과 라벨을 모은다.
for folder_path, class_id in sources:  # 원본 폴더를 하나씩 확인한다.
    folder_name = Path(folder_path).name  # 복사할 파일 이름 앞에 붙일 폴더 이름이다.
    images = sorted((root / folder_path).glob('*.jpg'))  # 사진을 정렬한다.
    if class_id is None and not images:  # 아직 카메라로 찍은 사진이 없으면 건너뛴다.
        print(f'사진 없음, 건너뜀: {folder_path}')  # 건너뛴 폴더를 알린다.
        continue
    if len(images) < 2:  # 학습과 검증에 각각 사진이 필요하다.
        raise SystemExit(f'사진이 부족합니다: {folder_name}')  # 잘못된 입력을 알린다.
    for image_path in images:  # 복사하기 전에 모든 라벨을 검사한다.
        label_path = image_path.with_suffix('.txt')  # 대응하는 라벨 경로다.
        if not label_path.is_file():  # 누락된 라벨을 확인한다.
            raise SystemExit(f'라벨이 없습니다: {image_path}')  # 누락 시 중단한다.
        rows = label_path.read_text().splitlines()  # 박스별 정답을 읽는다.
        if not rows:  # 작업이 끝나지 않은 빈 라벨을 확인한다.
            raise SystemExit(f'빈 라벨입니다: {label_path}')  # 빈 라벨을 알린다.
        for row in rows:  # 클래스와 정규화 좌표를 검사한다.
            values = row.split()  # 다섯 항목으로 나눈다.
            if (len(values) != 5 or values[0] not in class_ids  # 클래스 번호를 확인한다.
                    or class_id is not None and values[0] != str(class_id)):
                raise SystemExit(f'잘못된 라벨: {label_path}')  # 오류를 알린다.
            coords = [float(value) for value in values[1:]]  # 좌표를 숫자로 변환한다.
            if not all(0 <= value <= 1 for value in coords) or min(coords[2:]) <= 0:
                raise SystemExit(f'잘못된 좌표: {label_path}')  # 좌표 범위를 확인한다.
    boundary = len(images) - max(1, round(len(images) * 0.2))  # 마지막 약 20%를 검증에 쓴다.
    for index, image_path in enumerate(images):  # 클래스별 분할을 기록한다.
        pairs.append((folder_name, image_path, 'train' if index < boundary else 'val'))
for folder_name, image_path, split in pairs:  # 검사한 원본을 복사한다.
    for kind, source_path in [('images', image_path), ('labels', image_path.with_suffix('.txt'))]:
        destination = dataset / kind / split  # 저장할 분할 폴더다.
        destination.mkdir(parents=True, exist_ok=True)  # 폴더를 준비한다.
        name = f'{folder_name}_{source_path.name}'  # 같은 파일 이름이 덮어써지지 않게 영상 이름을 붙인다.
        shutil.copy2(source_path, destination / name)  # 원본을 보존하며 복사한다.

yaml_text = (  # YOLO에 데이터 위치와 클래스 이름을 알려줄 내용을 만든다.
    "# YAML은 설정을 '항목: 값'으로 적는 파일이며, # 뒤의 설명은 실행에 영향을 주지 않는다.\n"  # 형식을 설명한다.
    f'path: {dataset}  # 데이터셋의 기준 폴더다.\n'  # 데이터셋의 최상위 경로를 기록한다.
    'train: images/train  # path 아래의 학습 사진 폴더다. 세 클래스가 포함된다.\n'  # 학습 경로를 기록한다.
    'val: images/val  # path 아래의 검증 사진 폴더다. 세 클래스가 포함된다.\n'  # 검증 경로를 기록한다.
    '# 라벨 경로는 따로 적지 않는다. YOLO가 images를 labels로 바꿔 같은 이름의 txt를 찾는다.\n'  # 대응을 설명한다.
    'names:  # 모델이 구분할 클래스 번호와 이름을 적는다.\n'  # 클래스 목록 항목을 만든다.
    '  0: strawberry  # 라벨 txt의 첫 숫자 0은 딸기를 의미한다. 앞의 두 칸 들여쓰기를 유지한다.\n'  # 클래스를 기록한다.
)
yaml_text += '  1: car  # 라벨 번호 1은 자동차다.\n'  # 두 번째 클래스를 등록한다.
yaml_text += '  2: ball  # 라벨 번호 2는 공이다.\n'  # 세 번째 클래스를 등록한다.
(dataset / 'data.yaml').write_text(yaml_text, encoding='utf-8')  # 학습 설정 파일로 저장한다.
print('준비 완료:', {split: sum(p[2] == split for p in pairs) for split in ('train', 'val')})  # 처리 결과를 출력한다.
print(f'데이터 설정: {dataset / "data.yaml"}')  # 다음 학습 단계에서 사용할 경로를 출력한다.
