# **BÁO CÁO NGHIÊN CỨU KỸ THUẬT:** **THIẾT KẾ VÀ TRIỂN KHAI MODULE** **WELDING SEQUENCER CHO HỆ THỐNG** **ROBOT HÀN CÔNG NGHIỆP**
## **1. TỔNG QUAN VÀ PHẠM VI DỰ ÁN**
### **1.1. Giới thiệu**

Trong kỷ nguyên công nghiệp 4.0, tự động hóa quá trình hàn (Robotic Arc Welding) đóng vai
trò then chốt trong việc đảm bảo năng suất và chất lượng sản phẩm cơ khí. Tuy nhiên, sự
thành công của một hệ thống hàn robot không chỉ phụ thuộc vào cơ khí chính xác của cánh
tay robot hay công nghệ biến tần của nguồn hàn, mà còn phụ thuộc vào "bộ não" điều phối
giữa hai thực thể này: Module Trình tự Hàn (Welding Sequencer).


Báo cáo này được biên soạn dưới góc độ của một Kỹ sư Công nghệ Hàn (Welding Process
Engineer), nhằm đáp ứng yêu cầu thiết kế một module điều khiển cho hệ thống robot sử dụng
nguồn hàn MIG/MAG kỹ thuật số. Yêu cầu cốt lõi của dự án là robot phải đóng vai trò "Master"
(Chủ) trong việc quản lý trình tự thời gian (Timing Sequence) và logic vận hành, trong khi
nguồn hàn đóng vai trò "Slave" (Tớ) chịu trách nhiệm tính toán đường đặc tính Synergic (điện
áp/dòng điện) dựa trên các setpoint được gửi xuống.


Mục tiêu tối thượng là xây dựng một kiến trúc phần mềm và phần cứng đảm bảo quy trình: **Mở**


**khí** **Chờ (Pre-flow)** **Bật lửa** **Chờ tín hiệu Arc OK** **Bắt đầu di chuyển** diễn ra
chính xác, tin cậy và an toàn.

### **1.2. Định nghĩa Vấn đề và Yêu cầu Kỹ thuật**


Hệ thống hàn MIG/MAG (Metal Inert Gas / Metal Active Gas) là một quá trình phức tạp liên
quan đến sự phối hợp giữa hồ quang điện, chuyển dịch kim loại lỏng, và khí bảo vệ. Một trong
những nguyên nhân hàng đầu gây ra lỗi hàn trong môi trường robot (như rỗ khí, không ngấu,
cháy ngược) là do sự sai lệch về thời gian (timing mismatch) giữa chuyển động của robot và
trạng thái của hồ quang. [1 ]


Yêu cầu cụ thể từ người dùng nhấn mạnh vào việc kiểm soát trình tự thời gian chính xác mà
không can thiệp vào thuật toán vật lý hàn (Synergic). Điều này đặt ra các thách thức kỹ thuật
sau:


1.​ **Độ trễ tín hiệu (Signal Latency):** Phải xử lý độ trễ giữa lệnh Digital Output (DO) của

robot và phản hồi Digital Input (DI) từ nguồn hàn.


2.​ **Đồng bộ hóa chuyển động (Motion Synchronization):** Robot tuyệt đối không được di

chuyển khi hồ quang chưa hình thành ổn định ("Air Welding").
3.​ **Quản lý trạng thái (State Management):** Hệ thống phải hoạt động như một Máy trạng

thái hữu hạn (Finite State Machine - FSM) để ngăn chặn các chuyển đổi trạng thái không
hợp lệ.
4.​ **Tích hợp phần cứng:** Giao tiếp qua Digital I/O và Analog (0-10V) đòi hỏi các biện pháp

chống nhiễu và chuẩn hóa tín hiệu nghiêm ngặt.

## **2. CƠ SỞ LÝ THUYẾT VỀ QUY TRÌNH HÀN GMAW** **ROBOT**


Để thiết kế module WeldingSequencer hiệu quả, cần hiểu sâu sắc các hiện tượng vật lý diễn ra
trong từng mili-giây của quá trình khởi tạo hồ quang.

### **2.1. Vật lý của Khởi tạo Hồ quang (Arc Initiation Physics)**


Quá trình khởi tạo hồ quang không diễn ra tức thời. Khi robot gửi lệnh ARC_START, một chuỗi
các sự kiện vật lý phải xảy ra:


1.​ **Cấp dây (Wire Run-in):** Dây hàn được đẩy ra từ súng hàn với tốc độ chậm (Run-in

speed) để tiếp xúc với phôi. Nếu tốc độ này quá cao, dây sẽ đâm vào phôi gây nổ (blast);
nếu quá thấp, hồ quang sẽ chập chờn. [3 ]

2.​ **Ngắn mạch (Short Circuit):** Khi dây chạm phôi, điện áp giảm về gần 0V và dòng điện

tăng vọt (dòng ngắn mạch).
3.​ **Ion hóa và Bốc bay (Ionization & Vaporization):** Dòng điện lớn làm nóng chảy và bốc

hơi đoạn dây tiếp xúc, tạo ra cầu nối plasma dẫn điện (hồ quang).
4.​ **Ổn định (Stabilization):** Hệ thống điều khiển của nguồn hàn điều chỉnh dòng/áp để duy

trì cột hồ quang ổn định.


Module WeldingSequencer phải cung cấp đủ thời gian cho các bước này diễn ra trước khi cho
phép robot di chuyển. Nếu robot di chuyển trong giai đoạn ngắn mạch (bước 2), mối hàn sẽ bị
khuyết tật "Cold Lap" (chồng mép hàn không ngấu) tại điểm bắt đầu. [4 ]

### **2.2. Vai trò của Khí bảo vệ và Pre-flow**


Khí bảo vệ (như Argon,, hoặc hỗn hợp) có hai chức năng: bảo vệ kim loại nóng chảy
khỏi Oxy/Nitơ trong không khí và hỗ trợ mồi hồ quang.


●​ **Ion hóa:** Argon có thế năng ion hóa thấp hơn không khí, giúp hồ quang dễ mồi hơn.
●​ **Lưu lượng:** Cần một khoảng thời gian (Pre-flow time) để khí đẩy hết không khí trong ống

dẫn (torch cable) ra ngoài. Nếu mồi hồ quang ngay khi mở van khí, đoạn khí đầu tiên phun
ra vẫn lẫn không khí, gây rỗ khí (porosity) ngay tại điểm xuất phát. [6 ]


### **2.3. Hiện tượng Burnback và Crater Fill**

Ở cuối đường hàn, nếu cắt điện và dừng dây cùng lúc, dây hàn đang nóng chảy có xu hướng
dính chặt vào vũng hàn đang đông đặc hoặc dính ngược vào béc hàn (contact tip).


●​ **Crater Fill:** Giảm dòng điện từ từ để lấp đầy lõm cuối đường hàn, ngăn nứt chân chim.
●​ **Burnback:** Duy trì điện áp trong vài mili-giây sau khi ngừng cấp dây để đốt cháy đoạn

dây thừa, đảm bảo dây không dính vào phôi và có độ thò (stick-out) chuẩn cho lần mồi
sau. [8 ]


Dù nguồn hàn hiện đại thường có chức năng Burnback nội bộ, WeldingSequencer của robot
vẫn cần quản lý trạng thái này để đảm bảo robot không rút súng hàn ra khỏi vùng khí bảo vệ
quá sớm.

## **3. THIẾT KẾ KIẾN TRÚC HỆ THỐNG (SYSTEM** **ARCHITECTURE)**
### **3.1. Mô hình Phân cấp Master-Slave**


Hệ thống được thiết kế theo mô hình phân cấp, trong đó Robot Controller giữ vai trò điều phối
cấp cao.


●​ **Robot Controller (Sequencer):** Quản lý "KHI NÀO" (When) và "NẾU" (If). Ví dụ: "Khi nào

thì mở khí?", "Nếu không có hồ quang thì làm gì?".
●​ **Power Source (Synergic Core):** Quản lý "NHƯ THẾ NÀO" (How). Ví dụ: "Với 200A, điện

áp phải là bao nhiêu V để hồ quang ổn định?". Nguồn hàn tự động tra bảng Synergic
Curve để điều chỉnh. [1 ]

### **3.2. Giao diện Phần cứng (Hardware Interface Layer - HAL)**


Để thực hiện quy trình yêu cầu, chúng ta cần xác định danh sách tín hiệu I/O (I/O Map) chính
xác. Dựa trên các tài liệu kỹ thuật từ Fronius, Miller, và Panasonic [1], bảng dưới đây mô tả cấu
hình I/O tiêu chuẩn cho module này.


**3.2.1. Digital Outputs (Robot** **Nguồn hàn)**










|Tên Tín hiệu<br>(Code)|Loại|Chức năng|Mức Logic|Ghi chú Kỹ<br>thuật|
|---|---|---|---|---|
|**DO_GAS_VAL**<br>**VE**|Digital|Điều khiển van<br>khí điện từ<br>(Solenoid).|High = Open|Cần tách biệt<br>với Arc Start<br>để thực hiện|


|Col1|Col2|Col3|Col4|Pre-fol w độc<br>lập.12|
|---|---|---|---|---|
|**DO_ARC_STA**<br>**RT**|Digital|Lệnh khởi<br>động hàn<br>(Main<br>Contactor).|High = Weld|Kích hoạt chu<br>trình nội bộ<br>của nguồn hàn<br>(cấp dây, đóng<br>điện).|
|**DO_WIRE_INC**<br>**H **|Digital|Đẩy dây nhanh<br>(không hàn).|High = Feed|Dùng khi thay<br>cuộn dây hoặc<br>kiểm tra.|
|**DO_WIRE_RET**<br>**RACT**|Digital|Rút dây.|High = Retract|Hữu ích để xử<br>lý sự cố dính<br>dây hoặc căn<br>chỉnh<br>Stick-out.|
|**DO_ERROR_R**<br>**ESET**|Digital|Xóa lỗi nguồn<br>hàn.|Pulse (High<br> Low)|Dùng để reset<br>sau khi khắc<br>phục sự cố (ví<br>dụ: quá nhiệt).|
|**DO_JOB_BIT0**<br>**..3**|Digital|Chọn chương<br>trình hàn (Job<br>Select).|Binary/BCD|(Tùy chọn) Để<br>robot chọn<br>chế độ<br>Synergic (Vật<br>liệu/Khí/Đường<br>kính).|



**3.2.2. Digital Inputs (Nguồn hàn** **Robot)**










|Tên Tín hiệu<br>(Code)|Loại|Chức năng|Mức Logic|Ghi chú Kỹ<br>thuật|
|---|---|---|---|---|
|**DI_ARC_OK**|Digital|Xác nhận hồ<br>quang ổn định<br>(Current|High = OK|**Tín hiệu quan**<br>**trọng nhất.** <br>Cho phép|


|Col1|Col2|Detect).|Col4|robot bắt đầu<br>di chuyển.14|
|---|---|---|---|---|
|**DI_READY**|Digital|Nguồn hàn sẵn<br>sàng.|High = Ready|Nguồn đã bật,<br>không lỗi, sẵn<br>sàng nhận<br>lệnh.|
|**DI_WELD_ERR**<br>**OR**|Digital|Báo lỗi chung.|High = Fault|Quá nhiệt, mất<br>pha, hết nước<br>làm mát.1|
|**DI_WIRE_STU**<br>**CK**|Digital|Báo dính dây<br>(Anti-stick).|High = Stuck|Dây hàn dính<br>vào béc hàn<br>hoặc phôi sau<br>khi tắt hồ<br>quang.|
|**DI_GAS_FLO**<br>**W **|Digital|Cảm biến dòng<br>khí (Flow<br>sensor).|High = Flowing|(Khuyến nghị)<br>Feedback thực<br>tế dòng khí<br>thay vì chỉ tin<br>vào van điện<br>từ.|



**3.2.3. Analog Interface (Điều khiển Tham số)**


Mặc dù sử dụng chế độ Synergic, robot vẫn cần gửi tín hiệu tham chiếu để điều chỉnh năng
lượng hàn.


●​ **AO_WFS_REF (0-10V):** Tham chiếu tốc độ cấp dây (Wire Feed Speed) hoặc Dòng điện

(Amperage). Trong chế độ Synergic, tín hiệu này thường đại diện cho "Mức năng lượng"
(Power Level). Ví dụ: 5V = 150A. [1 ]

●​ **AO_VOLT_REF (0-10V):** Tham chiếu Điện áp hoặc Arc Length Correction (Trim). Trong

Synergic, 5V thường là mức chuẩn (0 trim), <5V là ngắn hồ quang, >5V là dài hồ quang. [4 ]

### **3.3. Sơ đồ Kết nối và Cách ly**


Môi trường hàn hồ quang tạo ra nhiễu điện từ (EMI) cực lớn do dòng điện cao tần và biến
thiên dòng lớn (di/dt).


●​ **Cách ly quang (Opto-isolation):** Tất cả các tín hiệu Digital I/O giữa robot và nguồn hàn

_bắt buộc_ phải đi qua bộ cách ly quang hoặc Relay trung gian để bảo vệ vi xử lý của robot
khỏi xung điện áp cao. [1 ]


●​ **Cáp Analog:** Sử dụng cáp xoắn đôi có bọc kim (Shielded Twisted Pair), với vỏ bọc nối đất

_chỉ tại một đầu_ (thường là phía tủ điều khiển robot) để tránh vòng lặp đất (Ground Loop)
gây sai lệch tín hiệu điều khiển 0-10V. [10 ]

## **4. THIẾT KẾ CHI TIẾT MODULE WELDING SEQUENCER**


Module WeldingSequencer được thiết kế dưới dạng một **Máy trạng thái hữu hạn (Finite**
**State Machine - FSM)** . Mô hình này đảm bảo tính tất định (determinism), nghĩa là với cùng
một đầu vào và trạng thái hiện tại, hệ thống luôn chuyển sang một trạng thái tiếp theo xác
định, loại bỏ các hành vi ngẫu nhiên nguy hiểm. [16 ]

### **4.1. Định nghĩa Các Trạng thái (States)**


Dựa trên quy trình yêu cầu, chúng ta định nghĩa 8 trạng thái cơ bản cho FSM:


1.​ **STATE_IDLE (0):** Hệ thống nghỉ, chờ lệnh CMD_START. Tất cả đầu ra tắt (trừ tín hiệu

Ready).


2.​ **STATE_PREFLOW (1):** Mở van khí, chờ thời gian .
3.​ **STATE_IGNITION (2):** Bật DO_ARC_START, chờ tín hiệu DI_ARC_OK. Đây là giai đoạn mồi

lửa.


4.​ **STATE_STABILIZE (3):** Hồ quang đã cháy, giữ yên robot trong thời gian để ổn định
vũng hàn.
5.​ **STATE_WELD (4):** Hồ quang ổn định, gửi cờ MOTION_ENABLE cho robot di chuyển.
6.​ **STATE_CRATER (5):** Nhận lệnh dừng, chuyển sang thông số Crater, giảm tốc độ.


7.​ **STATE_BURNBACK (6):** Tắt dây, giữ điện áp trong thời gian để chống dính.


8.​ **STATE_POSTFLOW (7):** Tắt hồ quang, giữ khí bảo vệ trong thời gian .
9.​ **STATE_FAULT (99):** Trạng thái lỗi (Mất hồ quang, dính dây, lỗi nguồn).

### **4.2. Biểu đồ Thời gian và Logic Chuyển đổi (Timing Diagram &** **Transition Logic)**


Dưới đây là phân tích chi tiết từng bước chuyển đổi, tích hợp các tham số thời gian và tín hiệu
I/O.


**Giai đoạn 1: Khởi động (Start Sequence)**


●​ **Từ IDLE sang PREFLOW:**

○​ _Điều kiện:_ Nhận lệnh CMD_START = TRUE.
○​ _Hành động:_ DO_GAS_VALVE = ON. Khởi động Timer t_state.
○​ _Logic:_ Tại sao phải chờ Pre-flow? Nếu ống dẫn khí dài 10m, đường kính trong 6mm,


thể tích khí cần đẩy là . Với lưu lượng 15


L/min (250 /s), cần ít nhất 1.2 giây để khí mới đến súng. Tuy nhiên, trong thực tế


sản xuất, ống luôn chứa sẵn khí (chỉ bị lẫn khí ở đầu vòi), nên thường đặt 0.1s 0.5s. [6 ]

○​ _Tham số:_ Param_PreFlowTime (Ví dụ: 0.3s).
●​ **Từ PREFLOW sang IGNITION:**

○​ _Điều kiện:_ t_state >= Param_PreFlowTime.
○​ _Hành động:_ DO_ARC_START = ON. Thiết lập Analog Setpoint = Param_Start_WFS /

Param_Start_Volts.
○​ _Hành động phụ:_ Reset Watchdog Timer cho việc mồi hồ quang.
○​ _Lưu ý:_ Robot vẫn đứng yên.
●​ **Từ IGNITION sang STABILIZE:**

○​ _Điều kiện:_ DI_ARC_OK == HIGH.
○​ _Hành động:_ Chuyển trạng thái. Bắt đầu Timer t_stab.
○​ _Xử lý lỗi:_ Nếu t_state > Param_IgnitionTimeout (ví dụ 3.0s) mà chưa có DI_ARC_OK,

chuyển sang STATE_FAULT với mã lỗi "Ignition Failure". [18 ]

●​ **Từ STABILIZE sang WELD:**

○​ _Điều kiện:_ t_stab >= Param_StabilizeTime (Ví dụ: 0.2s).
○​ _Hành động:_ Gửi tín hiệu MOTION_PERMIT = TRUE tới bộ điều khiển chuyển động của

robot. Chuyển Analog Setpoint sang Param_Weld_WFS / Param_Weld_Volts.
○​ _Ý nghĩa:_ Thời gian ổn định (Stabilize) giúp tạo vũng hàn ban đầu (weld pool) đủ nhiệt

trước khi di chuyển, tránh khuyết tật không ngấu đầu đường hàn.


**Giai đoạn 2: Hàn (Welding Process)**


●​ **Trong STATE_WELD:**

○​ _Giám sát:_ Liên tục kiểm tra DI_ARC_OK.
○​ _Sự cố:_ Nếu DI_ARC_OK chuyển xuống LOW (mất hồ quang đột ngột):

1.​ Ngắt MOTION_PERMIT ngay lập tức (Robot dừng).
2.​ Thử kích hoạt lại (Re-strike) trong 200ms.
3.​ Nếu không thành công, chuyển sang STATE_FAULT. [19 ]


**Giai đoạn 3: Kết thúc (End Sequence)**


●​ **Từ WELD sang CRATER:**

○​ _Điều kiện:_ Nhận lệnh CMD_STOP hoặc đi hết quỹ đạo hàn.
○​ _Hành động:_ Robot dừng hoặc giảm tốc độ di chuyển. Chuyển Analog Setpoint sang

Param_Crater_WFS (thường 60-80% trị số hàn) và Param_Crater_Volts. Giữ trạng thái
này trong Param_CraterTime. [8 ]

○​ _Mục đích:_ Giảm dòng từ từ giúp khí thoát khỏi vũng hàn đang đông đặc, ngăn chặn

vết nứt lõm (crater crack).
●​ **Từ CRATER sang BURNBACK:**

○​ _Điều kiện:_ t_state >= Param_CraterTime.
○​ _Hành động:_ Tắt dây hàn (Analog WFS = 0 hoặc DO_STOP_WIRE nếu có). **Vẫn giữ**


**DO_ARC_START = ON** .
○​ _Logic Burnback:_ Đây là điểm mâu thuẫn thường gặp. Một số nguồn hàn tự quản lý

Burnback. Nếu nguồn hàn tự quản lý, robot chỉ cần tắt DO_ARC_START và nguồn hàn
sẽ tự làm phần còn lại. Tuy nhiên, với yêu cầu "Kiểm soát chính xác thời gian" từ người
dùng, Sequencer nên hỗ trợ cả hai chế độ. Ở đây giả định chế độ Robot điều khiển:
Robot giữ Contactor đóng nhưng ngừng cấp dây trong Param_BurnbackTime (ví dụ
0.05s - 0.1s) để hồ quang đốt ngắn dây lại. [9 ]

○​ _Kết thúc Burnback:_ DO_ARC_START = OFF.
●​ **Từ BURNBACK sang POSTFLOW:**

○​ _Điều kiện:_ t_state >= Param_BurnbackTime.
○​ _Hành động:_ DO_ARC_START đã tắt. DO_GAS_VALVE vẫn giữ ON.
○​ _Thời gian:_ Chờ Param_PostFlowTime (Ví dụ 1.0s - 3.0s).
○​ _Mục đích:_ Bảo vệ điện cực vonfram (TIG) hoặc dây hàn nóng đỏ (MIG) và vũng hàn

khỏi bị oxy hóa khi nguội đi. [20 ]

●​ **Từ POSTFLOW về IDLE:**

○​ _Hành động:_ DO_GAS_VALVE = OFF. Reset toàn bộ biến nội bộ. Sẵn sàng cho chu trình

mới.

## **5. TRIỂN KHAI KỸ THUẬT VÀ THUẬT TOÁN** **(IMPLEMENTATION & ALGORITHMS)**


Phần này mô tả cấu trúc phần mềm bằng giả mã C++ (phổ biến trong phát triển controller
robot) để minh họa logic của WeldingSequencer.

### **5.1. Cấu trúc Lớp (Class Structure)**


C++


​


​

​

​

​

### **5.2. Thuật toán Vòng lặp Điều khiển (Control Loop Logic)**


Thuật toán này cần được gọi trong vòng lặp thời gian thực (Real-time loop) của robot, với chu
kỳ quét khoảng 5ms - 10ms để đảm bảo độ chính xác. [17 ]


**Logic Xử lý Tín hiệu Arc OK (Debouncing):**


Tín hiệu DI_ARC_OK thường rất nhiễu trong giai đoạn ngắn mạch (Short Circuit Transfer). Nếu


đọc thô (raw), FSM có thể nhảy trạng thái liên tục.


●​ _Giải pháp:_ Sử dụng bộ đếm hoặc bộ lọc thời gian.
●​ _Quy tắc:_ DI_ARC_OK chỉ được coi là HIGH nếu nó giữ mức 1 trong ít nhất 20ms liên tục.

Ngược lại, chỉ coi là mất hồ quang nếu nó ở mức 0 trong >50ms (để bỏ qua các lần tắt
ngắn mạch tự nhiên). [14 ]


**Chi tiết thuật toán chuyển trạng thái (Pseudocode):**


C++


​

​

​


​

​

​


break;​
​

## **6. XỬ LÝ LỖI VÀ PHỤC HỒI (ERROR HANDLING &** **RECOVERY)**


Một hệ thống tự động tốt được đánh giá qua khả năng xử lý sự cố chứ không chỉ là chạy trơn
tru khi mọi thứ hoàn hảo. Dưới đây là các chiến lược phục hồi cho các lỗi phổ biến được trích
xuất từ kinh nghiệm vận hành thực tế. [18 ]

### **6.1. Lỗi Mồi Hồ quang (Ignition Failure)**


**Hiện tượng:** Robot gửi lệnh ARC_START, dây chạy ra, chạm phôi nhưng không bắt lửa, hoặc
cháy vụt rồi tắt.


**Nguyên nhân:** Bề mặt phôi bẩn/sơn, tiếp xúc mass kém, dây hàn bị cắt cụt (không nhọn).


**Chiến lược Phục hồi (Auto-Recovery Logic):**


1.​ **Phát hiện:** Ignition Timeout kích hoạt.
2.​ **Hành động tức thời:** Ngắt DO_ARC_START, Ngắt DO_GAS.
3.​ **Quy trình Thử lại (Retry Routine):**

○​ Kích hoạt DO_WIRE_RETRACT trong 0.5s để rút dây về (tránh dây đâm cong vào

phôi).
○​ Nâng robot lên trục Z (+10mm).
○​ Di chuyển robot lùi lại một khoảng nhỏ (-5mm dọc đường hàn).
○​ Thử lại quy trình khởi động (tối đa 3 lần).
○​ Nếu vẫn thất bại: Dừng hệ thống và báo đèn đỏ cho người vận hành.

### **6.2. Lỗi Dính dây (Wire Stick / Burnback Error)**


**Hiện tượng:** Sau khi kết thúc hàn, dây hàn dính chặt vào vũng hàn đông đặc. Robot không thể
di chuyển đi chỗ khác.


**Nguy hiểm:** Nếu robot cố di chuyển, súng hàn sẽ bị bẻ cong, hoặc trục robot bị quá tải (Servo
Overload).


**Giải pháp:**


●​ Sử dụng tín hiệu DI_WIRE_STUCK từ nguồn hàn (nếu có).
●​ Nếu không có tín hiệu này, logic Sequencer có thể kiểm tra dòng điện: Nếu

DO_ARC_START đã tắt được 1s mà DI_ARC_OK (hoặc cảm biến dòng analog) vẫn báo có
dòng điện ngắn mạch, nghĩa là dây đang dính.
●​ **Hành động:** Khóa chuyển động robot (MOTION_INHIBIT). Gửi cảnh báo "WIRE STUCK".


Yêu cầu người vận hành cắt dây thủ công. Một số hệ thống cao cấp có thể thử phóng một
xung điện mạnh (Pulse) để làm nóng chảy điểm dính. [22 ]

## **7. GIAO DIỆN ĐIỆN VÀ TỐI ƯU HÓA TÍN HIỆU**


Để đảm bảo tính "chính xác về thời gian" như yêu cầu, phần cứng phải được thiết kế để giảm
thiểu nhiễu và độ trễ.

### **7.1. Độ trễ Hệ thống (System Latency)**


●​ **Van khí:** Van solenoid tiêu chuẩn mất 20ms-50ms để mở hoàn toàn. Sequencer phải

cộng thêm thời gian này vào Param_PreFlowTime.
●​ **Relay:** Relay cơ khí mất ~10ms. Nên sử dụng Relay bán dẫn (Solid State Relay - SSR) hoặc

Optocoupler để có thời gian đáp ứng <1ms.
●​ **Giao tiếp:** Nếu dùng I/O trực tiếp (Hardwired), độ trễ gần như bằng 0. Nếu dùng qua PLC


trung gian (Robot PLC Nguồn hàn), độ trễ giao tiếp (Bus cycle time) có thể lên tới
10-20ms. Với yêu cầu chính xác cao, khuyến nghị đấu nối trực tiếp I/O từ Robot vào
Nguồn hàn.

### **7.2. Bảng Dữ liệu Tối ưu hóa Tham số (Parameter Tuning Guide)**


Bảng dưới đây cung cấp các giá trị tham chiếu để thiết lập ban đầu cho Sequencer, giúp Kỹ sư
vận hành tiết kiệm thời gian thử nghiệm. [3 ]












|Tham số|Giá trị Thép<br>(Steel)|Giá trị Nhôm<br>(Alu)|Giá trị Inox<br>(SS)|Giải thích Tác<br>động|
|---|---|---|---|---|
|**Pre-fow**|0.3s|0.5s|0.4s|Nhôm cần bảo<br>vệ kỹ hơn do<br>dễ bị oxy hóa<br>nhanh.|
|**Ignition Time**|2.0s (Timeout)|2.0s|2.0s|Giới hạn an<br>toàn.|
|**Stabilize**|0.2s|0.1s|0.2s|Thép cần thời<br>gian nung<br>nóng ban đầu<br>lâu hơn nhôm.|
|**Crater Time**|0.5s|0.8s|0.6s|Nhôm có hệ số|


|Col1|Col2|Col3|Col4|co ngót nhiệt<br>lớn, cần Crater<br>lâu để tránh<br>nứt.|
|---|---|---|---|---|
|**Burnback**|0.08s|0.12s|0.08s|Dây nhôm<br>mềm và dẫn<br>nhiệt nhanh,<br>dễ bị chảy<br>ngược lên béc<br>hàn, cần<br>Burnback kỹ.|
|**Post-fow**|1.0s|3.0s|2.0s|Inox và Nhôm<br>cần khí sau lâu<br>để bề mặt mối<br>hàn sáng<br>bóng.|

## **8. KẾT LUẬN**

Việc thiết kế module WeldingSequencer cho hệ thống robot hàn MIG/MAG kỹ thuật số không
chỉ đơn thuần là lập trình Bật/Tắt tín hiệu, mà là sự tổng hòa giữa kiến thức vật lý hàn, lý thuyết
điều khiển tự động và kỹ thuật phần mềm thời gian thực.


Bản báo cáo này đã trình bày một giải pháp toàn diện, đáp ứng chính xác yêu cầu của người
dùng về việc tách biệt quản lý trình tự (do Robot đảm nhiệm) và quản lý năng lượng (do Nguồn
hàn đảm nhiệm). Với kiến trúc Máy trạng thái hữu hạn (FSM) được đề xuất, hệ thống đảm bảo:


1.​ **Chất lượng khởi đầu:** Loại bỏ rỗ khí nhờ logic Pre-flow chính xác.
2.​ **Độ tin cậy:** Ngăn chặn lỗi "Air Welding" nhờ cơ chế bắt tay DI_ARC_OK chặt chẽ.
3.​ **Tuổi thọ thiết bị:** Giảm thiểu dính dây và hỏng béc hàn nhờ quy trình Burnback/Crater

hợp lý.
4.​ **An toàn:** Tích hợp các quy trình xử lý lỗi tự động.


Giải pháp này hoàn toàn khả thi để triển khai trên các bộ điều khiển robot công nghiệp phổ
biến (như FANUC, Yaskawa, ABB, KUKA) sử dụng ngôn ngữ lập trình bậc cao hoặc logic PLC
tích hợp. Đây là nền tảng vững chắc để xây dựng một cell hàn robot hiệu suất cao, sẵn sàng
cho sản xuất hàng loạt.


**Works cited**


1.​ Robot Interface - Migatronic, accessed February 1, 2026,



[htps://www.migatronic.com/media/1386/50173004_manual_robot-interface-mig.](https://www.migatronic.com/media/1386/50173004_manual_robot-interface-mig.pdf)
[pdf](https://www.migatronic.com/media/1386/50173004_manual_robot-interface-mig.pdf)
2.​ Troubleshooting Robotic Welding Arc Faults - ABIBLOG, accessed February 1,



2026,
[htps://blog.binzel-abicor.com/usa/troubleshooting-robotic-welding-arc-faults](https://blog.binzel-abicor.com/usa/troubleshooting-robotic-welding-arc-faults)
3.​ Analog Interface, AD1359-1 | PDF | Welding | Construction - Scribd, accessed



February 1, 2026,
[htps://www.scribd.com/document/292527006/Analog-Interface-AD1359-1](https://www.scribd.com/document/292527006/Analog-Interface-AD1359-1)
4.​ Mr. Roboto: Welding fundamentals for managers - The Fabricator, accessed



February 1, 2026,
[htps://www.thefabricator.com/thefabricator/article/automationrobotics/mr-robot](https://www.thefabricator.com/thefabricator/article/automationrobotics/mr-roboto-welding-fundamentals-for-managers)
[o-welding-fundamentals-for-managers](https://www.thefabricator.com/thefabricator/article/automationrobotics/mr-roboto-welding-fundamentals-for-managers)
5.​ Digital v 'old school' settings? | Page 2 | MIG Welding Forum, accessed February 1,

2026,
[htps://www.mig-welding.co.uk/forum/threads/digital-v-old-school-setings.9718](https://www.mig-welding.co.uk/forum/threads/digital-v-old-school-settings.97180/page-2)
[0/page-2](https://www.mig-welding.co.uk/forum/threads/digital-v-old-school-settings.97180/page-2)
6.​ MIG Welding Gas Pressure Settings (with Charts) - Weld Guru, accessed



February 1, 2026, [htps://weldguru.com/mig-welding-gas-pressure/](https://weldguru.com/mig-welding-gas-pressure/)
7.​ How to Set Gas Pressure For MIG Welding With Charts And Explanations 


YesWelder, accessed February 1, 2026,
[htps://yeswelder.com/blogs/yeswelder/how-to-set-gas-pressure-for-mig-weldin](https://yeswelder.com/blogs/yeswelder/how-to-set-gas-pressure-for-mig-welding-with-charts-and-explanations)
[g-with-charts-and-explanations](https://yeswelder.com/blogs/yeswelder/how-to-set-gas-pressure-for-mig-welding-with-charts-and-explanations)
8.​ Welding Parameters Explained - YesWelder, accessed February 1, 2026,



[htps://yeswelder.com/blogs/yeswelder/welding-parameters-explained](https://yeswelder.com/blogs/yeswelder/welding-parameters-explained)
9.​ Burnback control | MIG Welding Forum, accessed February 1, 2026,



[htps://www.mig-welding.co.uk/forum/threads/burnback-control.46/](https://www.mig-welding.co.uk/forum/threads/burnback-control.46/)
10.​ Welding Burnback: What Is It & How to Adjust the Controls - UNIMIG, accessed



February 1, 2026,
[htps://unimig.com.au/blog/welding-burnback-what-is-it-and-how-to-adjust-the](https://unimig.com.au/blog/welding-burnback-what-is-it-and-how-to-adjust-the-controls)
[-controls](https://unimig.com.au/blog/welding-burnback-what-is-it-and-how-to-adjust-the-controls)
11.​ Adding Capability to Your Robotic Welding Process | Y-Blog - Yaskawa Motoman,

accessed February 1, 2026,
[htps://www.motoman.com/en-us/about/blog/adding-capability-to-your-robotic-](https://www.motoman.com/en-us/about/blog/adding-capability-to-your-robotic-welding-process)
[welding-process](https://www.motoman.com/en-us/about/blog/adding-capability-to-your-robotic-welding-process)
12.​ Robotic Interface II - Miller Welding, accessed February 1, 2026,



[htps://www.millerwelds.com/fles/owners-manuals/o172324d_mil.pdf](https://www.millerwelds.com/files/owners-manuals/o172324d_mil.pdf)
13.​ YA-1NA***/ YA-1PA***, accessed February 1, 2026,



[htps://icdn.tradew.com/fle/201606/1569362/pdf/7593147.pdf](https://icdn.tradew.com/file/201606/1569362/pdf/7593147.pdf)
14.​ User Information - Fronius International, accessed February 1, 2026,



[htps://www.fronius.com/~/downloads/Perfect%20Welding/User%20Information/](https://www.fronius.com/~/downloads/Perfect%20Welding/User%20Information/42%2C0426%2C0227%2CEA.pdf)
[42%2C0426%2C0227%2CEA.pdf](https://www.fronius.com/~/downloads/Perfect%20Welding/User%20Information/42%2C0426%2C0227%2CEA.pdf)
15.​ analysis of welding parameters in gas metal arc welding by a welding robot 


Middle East Technical University, accessed February 1, 2026,
[htps://open.metu.edu.tr/bitstream/handle/11511/15987/index.pdf](https://open.metu.edu.tr/bitstream/handle/11511/15987/index.pdf)


16.​ Draw a finite state machine for a welding machine - Electronics Stack Exchange,



accessed February 1, 2026,
[htps://electronics.stackexchange.com/questions/122737/draw-a-fnite-state-mac](https://electronics.stackexchange.com/questions/122737/draw-a-finite-state-machine-for-a-welding-machine)
[hine-for-a-welding-machine](https://electronics.stackexchange.com/questions/122737/draw-a-finite-state-machine-for-a-welding-machine)
17.​ Implementing a Real-Time State Machine in Modern C++ - honeytreeLabs,



accessed February 1, 2026,
[htps://honeytreelabs.com/posts/real-time-state-machine-in-cpp/](https://honeytreelabs.com/posts/real-time-state-machine-in-cpp/)
18.​ Improved Response to Welding Arc Failure - Beacon Platform - Hirebotics,



accessed February 1, 2026,
[htps://help.hirebotics.com/en/articles/10080737-improved-response-to-welding](https://help.hirebotics.com/en/articles/10080737-improved-response-to-welding-arc-failure)
[-arc-failure](https://help.hirebotics.com/en/articles/10080737-improved-response-to-welding-arc-failure)
19.​ Dealing with Arc Start Failures - Yaskawa Knowledge Center, accessed February 1,



2026,
[htps://knowledge.motoman.com/hc/en-us/articles/4408792362903-Dealing-with](https://knowledge.motoman.com/hc/en-us/articles/4408792362903-Dealing-with-Arc-Start-Failures)
[-Arc-Start-Failures](https://knowledge.motoman.com/hc/en-us/articles/4408792362903-Dealing-with-Arc-Start-Failures)
20.​ OM-154 145B Robotic Interface Control Gas/Current Sensing Control - Miller



Welding, accessed February 1, 2026,
[htps://www.millerwelds.com/fles/owners-manuals/o154145b.pdf](https://www.millerwelds.com/files/owners-manuals/o154145b.pdf)
21.​ Basic Fault Recovery Guide | Robots.com, accessed February 1, 2026,



[htps://www.robots.com/articles/basic-fault-recovery-a-step-by-step-guide-for-](https://www.robots.com/articles/basic-fault-recovery-a-step-by-step-guide-for-successful-execution)
[successful-execution](https://www.robots.com/articles/basic-fault-recovery-a-step-by-step-guide-for-successful-execution)
22.​ Solving Welding Burnbacks in Robotic Applications - ABIBLOG, accessed

February 1, 2026,
[htps://blog.binzel-abicor.com/usa/solving-welding-burnbacks-in-robotic-applica](https://blog.binzel-abicor.com/usa/solving-welding-burnbacks-in-robotic-applications)
[tions](https://blog.binzel-abicor.com/usa/solving-welding-burnbacks-in-robotic-applications)
23.​ LGA Interface - Lincoln Electric, accessed February 1, 2026,



[htps://assets.lincolnelectric.com/assets/EU/OperatorManuals/IM3056rev01-ENG.](https://assets.lincolnelectric.com/assets/EU/OperatorManuals/IM3056rev01-ENG.pdf)
[pdf](https://assets.lincolnelectric.com/assets/EU/OperatorManuals/IM3056rev01-ENG.pdf)


