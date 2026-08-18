# 자작 드론 제작 시간·비용 예비 산정

이 문서는 발표자료의 `1대와 10대 제작 규모` 장표에 쓰는 계획 가정과 가격 근거를
기록한다. 저장소에는 완전한 제조 BOM, PCB 생산파일, 슬라이서 결과, 작업시간 기록이
없다. 따라서 아래 값은 실적 원가가 아니라 부품 재고가 있다는 가정에서 만든
일정 계획치와 가격이 확인되는 부품의 하한이다. 화면에 쓰는 값의 기계 판독 원본은
[`production_estimate.json`](production_estimate.json)이다.

## 저장소에서 확인한 구성

- ESP32-S3-N16R8 1개
- ICM-42670-P 2개, BMM350 1개
- EMAX BLHeli 30A ESC 4개
- 1750KV급 모터 4개
- T-MOTOR T4944-3 프로펠러 1세트
- ECOFLUX 4S 3000mAh 배터리 1개
- 3D 프린팅 암 4개와 상·하판
- 선택 장착 예정인 3901-L0X 1개

근거는 [`index.html`](index.html)의 부품 구성 장표와
[`dual_imu_cascade_pwm.ino`](../../../firmware/flight/dual_imu_cascade_pwm/dual_imu_cascade_pwm.ino)의
현행 모터·센서 객체이다. 실물 모터 표기와 발표자료의 모델명이 일치하는지는 구매
기록이 없어 확인하지 못했다. 자체 PCB의 정확한 리비전별 BOM도 저장소에 없다.

## 제작시간

| 규모 | 프린터 점유 | 직접 작업 | 프린터 1대 기준 경과 |
|---|---:|---:|---:|
| 1대 | 24~48시간 | 6~10인시 | 약 2~3일 |
| 10대 | 240~480시간 | 47~74인시 | 약 10~20일 |

다음 조건을 전제로 한 일정 계획치이다.

- 부품 재고 확보, 기구·회로·펌웨어 설계 완료
- 프린터 한 대에서 순차 출력
- 기체당 프린트 소재 0.4~0.8kg, 첫 형상 출력 성공
- PCB 조립 완료 기준의 납땜·조립·펌웨어 기록·교정·벤치 점검
- 자유비행 튜닝, 반복 비행 검증, 재출력, 납기 대기는 제외

10대의 직접 작업은 공정 묶음 처리에 따른 계획치이지만, 기체별 센서·모터 방향과
안전 점검은 생략하지 않는다. 프린터 두 대이면 출력 경과는 약 5~10일, 네 대이면
약 2.5~5일로 줄어들 수 있다. 사람 작업과 안전 점검은 프린터 수만 늘려도 같은
비율로 줄지 않는다.

## 가격이 확인되는 핵심 부품

조회 가격은 판매처와 수량에 따라 바뀔 수 있다. 해외 판매가는 배송·관부가세·환전
수수료를 제외했다.

| 항목 | 1대 범위 | 산정 근거 |
|---|---:|---|
| 모터 4개 | 69,000~82,000원 | DYS 1750KV 4개 판매가와 발표자료의 CCRC 1750KV 대체 판매가 범위 |
| EMAX BLHeli 30A ESC 4개 | 51,200~175,400원 | 국내 단가와 EMAX 공식 단가 범위. 실제 SKU·4S 적합성 재확인 필요 |
| ESP32-S3 + ICM-42670-P 2개 + BMM350 | 약 22,700원 | DigiKey 단품가 기준 핵심 칩 소계 |
| 프레임 재료 | 11,200~22,400원 | PLA 28,000원/kg, 0.4~0.8kg 계획 가정 |
| 배터리 + 프로펠러 | 약 68,000원 | ECOFLUX 배터리와 T4944-3 1세트 참고가 |
| **가격 확인 가능 부품 소계** | **약 222,100~370,500원** | PCB·배선·체결·배송·인건비 제외 |
| 선택 3901-L0X | **대당 +48,000~49,000원** | 국내 판매처 참고가 |

10대는 수량 할인이나 양산 효과를 가정하지 않고 같은 범위를 단순 확장해 약
2,221,000~3,705,000원으로 본다. 이는 완성기 총원가가 아니다. 다음 항목은 규격·수량
또는 생산파일이 없어 별도 견적이 필요하다.

- 자체 FC PCB, 전원·보호 회로, 부품 실장
- 배선, 커넥터, XT60, 수축튜브, 나사, 인서트, 스탠드오프
- 배송·관부가세·환전 수수료
- 인건비, 장비 감가, 실패 출력과 재작업

## 가격 출처

- DYS 모터: <https://www.dysrc.hk/>
- CCRC 1750KV급 모터 대체 판매가: <https://rcdrone.top/products/ccrc-sunhey-2004-brushless-motor>
- EMAX 30A ESC 국내 판매가: <https://ercmall.co.kr/product/esc-emax-bl-heli-30a/4545/>
- EMAX 30A ESC 공식 판매가: <https://shop.emaxmodel.com/products/blheli-series-30a-esc-oneshot-available>
- ESP32-S3-N16R8: <https://www.digikey.kr/ko/products/detail/espressif-systems/ESP32-S3-WROOM-1U-N16R8/16162641>
- ICM-42670-P: <https://www.digikey.kr/ko/products/detail/tdk-invensense/ICM-42670-P/14319524>
- BMM350: <https://www.digikey.kr/en/products/detail/bosch-sensortec/BMM350/17827582>
- T-MOTOR T4944-3: <https://phaserfpv.com.au/products/t-motor-t4944-49-propeller-2cw-2ccw>
- ECOFLUX 4S 3000mAh: <https://helsel.co.kr/product/ecoflux-3000mah-4s-148v-120c-xt60-%EB%A6%AC%ED%8A%AC%ED%8F%B4%EB%A6%AC%EB%A8%B8-%EB%B0%B0%ED%84%B0%EB%A6%AC-%EC%97%90%EC%BD%94%ED%94%8C%EB%9F%AD%EC%8A%A4/33018/category/1/display/13/>
- Bambu PLA Basic: <https://kr.store.bambulab.com/products/pla-basic-filament>
- 3901-L0X 국내 판매가: <https://xcopter.com/product/%EB%A7%88%ED%85%8D-matek-%EC%98%B5%ED%8B%B0%EC%BB%AC-%EB%9D%BC%EC%9D%B4%EB%8B%A4-%EC%84%BC%EC%84%9C-3901-l0x/32509/>, <https://www.rcbank.co.kr/shop/goods/goods_view.php?goodsno=34212>

발표 화면에서는 판매처·환율·개별 단가를 나열하지 않는다. `예비 산정`, 가정,
미포함 항목을 함께 표시하고, 실제 제작 전에는 모터 SKU, PCB 리비전, 슬라이서의
재료량·시간, PCB 실장 견적을 다시 확정한다.
