import pandas as pd
import matplotlib.pyplot as plt
import os
import glob
import sys

# ==========================================
# 1. 가장 최근 CSV 파일 자동 선택
# ==========================================
list_of_files = glob.glob('*.csv') 

if not list_of_files:
    print("❌ 현재 폴더에 CSV 파일이 없습니다.")
    sys.exit()

# 생성 시간 순으로 정렬 -> 가장 최신 파일 선택
latest_file = max(list_of_files, key=os.path.getctime)
file_path = latest_file

# ==========================================
# 2. 데이터 읽기 및 전처리
# ==========================================
try:
    print(f"📂 분석 대상: {file_path}")
    
    # CSV 읽기
    df = pd.read_csv(file_path, skipinitialspace=True)
    df.columns = df.columns.str.strip() # 컬럼명 공백 제거
    
    if df.empty:
        print("❌ 파일이 비어있습니다.")
        sys.exit()

    print(f"✅ 데이터 로딩 완료! ({len(df)} 개 샘플)")

except Exception as e:
    print(f"❌ 파일 읽기 오류: {e}")
    sys.exit()

# ==========================================
# 3. 데이터 통계 출력 (상세 분석용)
# ==========================================
print("\n" + "="*60)
print("📊 비행 데이터 요약 통계")
print("="*60)

# 보고 싶은 컬럼들 정의
cols_attitude = ['Roll', 'Pitch', 'Yaw']
cols_gyro = ['Gyro_X', 'Gyro_Y', 'Gyro_Z']
cols_accel = ['Accel_X', 'Accel_Y', 'Accel_Z']
cols_input = ['Throttle']

# 존재하는 컬럼만 필터링
all_targets = cols_attitude + cols_gyro + cols_accel + cols_input
available_cols = [c for c in all_targets if c in df.columns]

# 통계 출력 (소수점 2자리)
print(df[available_cols].describe().round(2))
print("="*60)

# ==========================================
# 4. 그래프 그리기 (3단 구성)
# ==========================================
# 스타일 설정
plt.style.use('seaborn-v0_8-darkgrid' if 'seaborn-v0_8-darkgrid' in plt.style.available else 'default')

# 3개의 서브플롯 생성 (높이 12인치)
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12), sharex=True)
fig.suptitle(f'Flight Analysis: {os.path.basename(file_path)}', fontsize=16, fontweight='bold')

x_axis = range(len(df))

# --- [1] Attitude (자세) ---
if 'Roll' in df.columns:
    ax1.plot(x_axis, df['Roll'], label='Roll', color='red', linewidth=1.5)
if 'Pitch' in df.columns:
    ax1.plot(x_axis, df['Pitch'], label='Pitch', color='blue', linewidth=1.5)
if 'Yaw' in df.columns:
    ax1.plot(x_axis, df['Yaw'], label='Yaw', color='green', linewidth=1.5, linestyle='--')

ax1.axhline(0, color='black', linestyle=':', alpha=0.5) # 0도 기준선
ax1.set_ylabel('Angle (deg)', fontsize=12)
ax1.set_title('1. Attitude Response', fontsize=14)
ax1.legend(loc='upper right')
ax1.grid(True, linestyle='--', alpha=0.7)
ax1.set_ylim(-60, 60) # 각도 범위 고정 (보기 편하게)

# --- [2] Gyroscope Raw (진동 분석) ---
if 'Gyro_X' in df.columns:
    ax2.plot(x_axis, df['Gyro_X'], label='Gyro X', color='red', alpha=0.6, linewidth=1)
if 'Gyro_Y' in df.columns:
    ax2.plot(x_axis, df['Gyro_Y'], label='Gyro Y', color='blue', alpha=0.6, linewidth=1)
if 'Gyro_Z' in df.columns:
    ax2.plot(x_axis, df['Gyro_Z'], label='Gyro Z', color='green', alpha=0.6, linewidth=1)

ax2.set_ylabel('Angular Rate (dps)', fontsize=12)
ax2.set_title('2. Gyroscope Raw (Vibration Check)', fontsize=14)
ax2.legend(loc='upper right')
ax2.grid(True, linestyle='--', alpha=0.7)

# --- [3] Accelerometer & Throttle (가속도 및 입력) ---
# 가속도는 왼쪽 Y축, 스로틀은 오른쪽 Y축 사용
if 'Accel_X' in df.columns:
    ax3.plot(x_axis, df['Accel_X'], label='Accel X', color='red', alpha=0.5, linewidth=1)
if 'Accel_Y' in df.columns:
    ax3.plot(x_axis, df['Accel_Y'], label='Accel Y', color='blue', alpha=0.5, linewidth=1)
if 'Accel_Z' in df.columns:
    ax3.plot(x_axis, df['Accel_Z'], label='Accel Z', color='green', alpha=0.5, linewidth=1)

ax3.set_ylabel('Acceleration (g)', fontsize=12)
ax3.set_ylim(-2.0, 2.0) # 가속도 보기 편하게 고정
ax3.legend(loc='upper left')
ax3.grid(True, linestyle='--', alpha=0.7)

# 스로틀 (오른쪽 축)
ax3_right = ax3.twinx()
if 'Throttle' in df.columns:
    ax3_right.plot(x_axis, df['Throttle'], label='Throttle', color='orange', linewidth=2, linestyle='-')
    ax3_right.set_ylabel('Throttle (PWM)', color='orange', fontsize=12)
    ax3_right.tick_params(axis='y', labelcolor='orange')
    ax3_right.legend(loc='upper right')
    ax3_right.set_ylim(1000, 2000)

ax3.set_title('3. Accelerometer & Throttle Input', fontsize=14)
ax3.set_xlabel('Sample Count', fontsize=12)

# 레이아웃 조정 및 표시
plt.tight_layout()
plt.show()