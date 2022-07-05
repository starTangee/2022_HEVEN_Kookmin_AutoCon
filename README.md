# 2022_Kookmin_AutoCon

2022 HEVEN 국민대학교 자율주행 경진대회 Repository (본선)<br>
*예선과제는 [여기](https://github.com/jhforstudy/2022_HEVEN_Kookmin_AutoCon_YS)로*

## 1. 본선과제 요약

1) **오전: 고속주행 경기**<br>
* 자율주행 모형차가 트랙을 시계 반대방향으로 3랩을 주행하여야 함.

2) **오후: 미션수행 경기**<br>
* 자율주행 모형차가 트랙을 시계 반대방향으로 1랩을 주행하여야 함.
* 트랙에 설치된 미션을 순서대로 하나씩 수행하여야 하며, 미션을 하나씩
수행할 때마다 미션별 기본점수에서 5장에 규정한 “미션수행 패널티
규정표”에 의한 벌점을 차감한 점수를 획득하게 됨. 

3) **최종 점수 합산**<br>
* 고속주행 경기는 시간기록, 미션수행 경기는 총점으로 계산함.
* 고속주행 경기 등수 + 미션수행 경기 등수의 합계가 가장 적은 팀이 최종
우승을 차지함. 

## 2. 최종 평가

자세한 점수 합산 방법은 규정집을 참고

## 3. 본선과제 규정집 & 안내

[국민대-2022-자율주행-경진대회-규정집-v1.0.pdf](https://github.com/jhforstudy/2022_HEVEN_Kookmin_AutoCon/files/9040245/-2022-.-.-.-v1.0.pdf)

[5회 자율주행경진대회-본선-안내.pdf](https://github.com/jhforstudy/2022_HEVEN_Kookmin_AutoCon/files/9040246/5.-.-.pdf)

## 4. 실행 방법

TBC

## 5. 센서별 호출 방법

TBC

## 6. 센서별 rosbag 파일

카메라 : [다운로드](https://drive.google.com/file/d/19e_oFJ1TOnnhaJg_sLq5L76Wk2-rTS7m/view?usp=sharing)<br>
LiDAR, IMU, 초음파 센서 : ``bagfiles`` 디렉터리 참고

## 7. 코드 다운로드 방법
### 터미널을 통해 다운로드
#### 1. 터미널을 열고, 파일을 담을 디렉터리 생성
```
mkdir kookmin_ws
cd kookmin_ws/
```

#### 2. 현재 레포지토리를 clone
```
git clone https://github.com/jhforstudy/2022_HEVEN_Kookmin_AutoCon.git
```

#### 3. clone한 폴더의 이름을 "src"로 변경 (패키지를 빌드하기 위해)
```
mv 2022_HEVEN_Kookmin_AutoCon/ src
```

#### 4. 패키지 빌드
```
catkin_make
```

### gitkraken을 이용해 다운로드
git 협업을 위해 gitkraken을 이용하는 것을 추천드립니다.
