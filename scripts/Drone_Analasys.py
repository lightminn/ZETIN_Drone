import pandas as pd
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import filedialog
import os

# ==========================================
# 1. 파일 선택 (GUI 창 열기)
# ==========================================
root = tk.Tk()
root.withdraw() # 빈 창 숨기기

print("📂 분석할 CSV 파일을 선택하세요...")
file_path = filedialog.askopenfilename(
    title="비행 로그 파일 선택",
    filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
)

if not file_path:
    print("❌ 파일이 선택되지 않았습니다.")
    exit()

# ==========================================
# 2. 데이터 읽기 및 전처리
# ==========================================
try:
    # CSV 읽기 (공백 제거 등 처리)
    df = pd.read_csv(file_path, skipinitialspace=True)
    
    # 컬럼 이름 공백 제거 (혹시 모를 에러 방지)
    df.columns = df.columns.str.strip()
    
    # 데이터가 비었는지 확인
    if df.empty:
        print("❌ 파일이 비어있습니다.")
        exit()

    print(f"\n✅ '{os.path.basename(file_path)}' 로딩 완료! ({len(df)} 개 데이터)")

except Exception as e:
    print(f"❌ 파일 읽기 오류: {e}")
    exit()

# ==========================================
# 3. 데이터 통계 출력 (튜닝용)
# ==========================================
print("\n" + "="*40)
print("📊 비행 데이터 요약 통계")
print("="*40)
print(df[['Roll', 'Pitch', 'Yaw', 'Throttle']].describe().round(2))
print("="*40)

# ==========================================
# 4. 그래프 그리기
# ==========================================
# 스타일 설정 (격자, 크기)
plt.style.use('seaborn-v0_8-darkgrid' if 'seaborn-v0_8-darkgrid' in plt.style.available else 'default')
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

fig.suptitle(f'Flight Log Analysis: {os.path.basename(file_path)}', fontsize=16)

# X축 생성 (샘플 개수 or 시간)
x_axis = range(len(df))

# --- [Subplot 1] Roll & Pitch (자세) ---
ax1.plot(x_axis, df['Roll'], label='Roll', color='red', linewidth=1.5)
ax1.plot(x_axis, df['Pitch'], label='Pitch', color='blue', linewidth=1.5)
ax1.axhline(0, color='black', linestyle='--', alpha=0.5) # 0도 기준선
ax1.set_ylabel('Angle (deg)', fontsize=12)
ax1.set_title('Attitude (Roll / Pitch)', fontsize=14)
ax1.legend(loc='upper right')
ax1.grid(True, which='both', linestyle='--', alpha=0.7)

# --- [Subplot 2] Yaw (헤딩) ---
ax2.plot(x_axis, df['Yaw'], label='Yaw', color='green', linewidth=1.5)
ax2.set_ylabel('Heading (deg)', fontsize=12)
ax2.set_title('Heading (Yaw)', fontsize=14)
ax2.legend(loc='upper right')
ax2.grid(True, which='both', linestyle='--', alpha=0.7)

# --- [Subplot 3] Throttle (출력) ---
ax3.plot(x_axis, df['Throttle'], label='Throttle', color='orange', linewidth=1.5)
ax3.set_ylabel('PWM Value', fontsize=12)
ax3.set_xlabel('Sample Count', fontsize=12)
ax3.set_title('Throttle Input', fontsize=14)
ax3.legend(loc='upper right')
ax3.grid(True, which='both', linestyle='--', alpha=0.7)

# 그래프 간격 조정 및 표시
plt.tight_layout()
plt.show()