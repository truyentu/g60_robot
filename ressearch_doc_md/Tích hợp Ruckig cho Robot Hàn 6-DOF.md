# **Báo cáo Nghiên cứu Kỹ thuật: Kiến trúc** **Tích hợp Ruckig OTG và Robotics Library** **(RL) cho Bộ Điều khiển Robot Hàn 6-DOF**
## **Tóm tắt Điều hành**

Báo cáo này trình bày một phân tích toàn diện và chiến lược thực thi kỹ thuật nhằm tích hợp
thư viện tạo quỹ đạo trực tuyến (Online Trajectory Generation - OTG) Ruckig (Phiên bản Cộng
đồng) vào kiến trúc điều khiển robot hàn 6 bậc tự do (6-DOF). Mục tiêu cốt lõi là giải quyết
các yêu cầu về chuyển động nội suy tuyến tính (MOVL), đảm bảo độ mượt mà của chuyển
động thông qua giới hạn đạo hàm bậc ba (Jerk/S-Curve), và khả năng phản hồi thời gian thực
(Real-time) đối với các tín hiệu cảm biến cho ứng dụng dò đường hàn (Seam Tracking).


Dựa trên nền tảng toán học cốt lõi (Core Math) từ Robotics Library (RL) để giải bài toán Động
học Nghịch (Inverse Kinematics - IK), báo cáo đề xuất một kiến trúc luồng dữ liệu (Data Flow)
tối ưu, trong đó Ruckig đóng vai trò là bộ nội suy Cartesian thời gian thực. Phân tích chỉ ra
rằng để đạt được chuyển động thẳng (Straight Line) chính xác trong không gian 3D, việc sử
dụng chế độ "Đồng bộ hóa Pha" (Phase Synchronization) của Ruckig là bắt buộc. Đồng thời,
để vượt qua hạn chế về tính toán điểm trung gian (intermediate waypoints) trong phiên bản
Cộng đồng, báo cáo xây dựng một cơ chế cập nhật trạng thái mục tiêu liên tục ("Arbitrary
Target States") để mô phỏng hành vi Seam Tracking với độ trễ tối thiểu.

## **1. Cơ sở Lý thuyết và Đặt vấn đề trong Điều khiển** **Robot Hàn**
### **1.1. Tầm quan trọng của Đạo hàm bậc ba (Jerk) trong Công nghệ Hàn** **Hồ quang**


Trong lĩnh vực hàn hồ quang bằng robot (GMAW/TIG), chất lượng mối hàn không chỉ phụ
thuộc vào các thông số nguồn hàn (dòng điện, điện áp) mà còn chịu ảnh hưởng trực tiếp bởi
độ ổn định của chuyển động mỏ hàn. Các bộ điều khiển truyền thống thường sử dụng biên
dạng vận tốc hình thang (Trapezoidal Velocity Profile), chỉ giới hạn vận tốc và gia tốc (đạo hàm
bậc hai). Mặc dù đơn giản về mặt tính toán, phương pháp này cho phép gia tốc thay đổi tức
thời (step change), dẫn đến giá trị Jerk (đạo hàm bậc ba của vị trí) tiến tới vô cùng tại các
điểm chuyển tiếp.


Đối với cơ cấu cơ khí của robot 6 trục, Jerk cao gây ra các rung động tần số cao tại khâu tác
động cuối (End-Effector/TCP). Trong quá trình hàn, sự rung động này dẫn đến:


1.​ **Biến động vũng hàn (Weld Pool Instability):** Kim loại lỏng bị dao động, gây ra các

khuyết tật như cháy chân (undercut) hoặc vân hàn không đều.
2.​ **Sai lệch quỹ đạo tức thời:** Ngay cả khi robot đi đúng đường danh định, rung động có

thể làm điểm tiếp xúc dây hàn (Wire Stick-out) dao động, thay đổi dòng hàn thực tế.
3.​ **Hao mòn cơ khí:** Các hộp số giảm tốc (như Harmonic Drive hay Cycloidal) chịu ứng suất

lớn, làm giảm tuổi thọ robot.


Ruckig giải quyết vấn đề này bằng cách tạo ra các quỹ đạo giới hạn Jerk (Jerk-limited), hay


còn gọi là biên dạng S-Curve. [1] Bằng cách kiểm soát, gia tốc thay đổi tuyến tính thay vì
nhảy bậc, triệt tiêu rung động và đảm bảo mỏ hàn di chuyển "mượt" như chất lỏng, yếu tố tiên
quyết cho mối hàn chất lượng cao.

### **1.2. Sự khác biệt giữa Offline Planning và Online Trajectory Generation** **(OTG)**


Hiểu rõ sự khác biệt này là chìa khóa để thiết kế kiến trúc điều khiển đúng đắn.


●​ **Offline Planning (Lập kế hoạch ngoại tuyến):** Các thư viện như OMPL (trong MoveIt)

tính toán toàn bộ quỹ đạo từ điểm A đến B trước khi robot bắt đầu di chuyển. Phương
pháp này tối ưu cho việc tránh vật cản trong môi trường tĩnh nhưng hoàn toàn bất lực
trước các thay đổi động.
●​ **Online Trajectory Generation (OTG - Ruckig):** Ruckig tính toán trạng thái tiếp theo của

robot ngay trong chu kỳ điều khiển (ví dụ: mỗi 1ms). Nó nhận đầu vào là trạng thái hiện tại


( ) và trạng thái mục tiêu ( ) để đưa ra trạng thái kế tiếp (


) tuân thủ các giới hạn động học. [2 ]


**Thách thức với Ruckig Community Version:** Phiên bản Cộng đồng của Ruckig có một giới
hạn quan trọng: nó không hỗ trợ tính toán thời gian thực cho danh sách các điểm trung gian
(Waypoints). [4] Nếu người dùng nạp một danh sách điểm, nó sẽ chuyển sang tính toán offline
hoặc API đám mây, không phù hợp cho vòng điều khiển 1ms. Do đó, kiến trúc đề xuất phải sử
dụng cơ chế "State-to-State" (từ Trạng thái đến Trạng thái) và cập nhật mục tiêu liên tục để
xử lý các đường hàn phức tạp hoặc tracking.

## **2. Kiến trúc Tích hợp và Luồng Dữ liệu (Data Flow) Tối** **ưu**


Để tích hợp Ruckig (xử lý quỹ đạo) với Robotics Library (xử lý động học) trong một hệ thống
thời gian thực, chúng ta cần xác định không gian làm việc của OTG.


### **2.1. Quyết định Chiến lược: Cartesian OTG hay Joint Space OTG?**

Đây là quyết định quan trọng nhất ảnh hưởng đến khả năng thực hiện lệnh MOVL (đi thẳng).












|Đặc điểm|Joint Space OTG (Nội suy<br>Khớp)|Cartesian Space OTG<br>(Nội suy Đề các)|
|---|---|---|
|**Đầu vào Ruckig**|Góc khớp (<br>)|Tọa độ TCP (<br>)|
|**Đường đi TCP**|Đường cong (vòng cung)<br>trong không gian|Đường thẳng (nếu dùng<br>đúng sync)|
|**Singularity**|An toàn, không bao giờ gặp<br>điểm kỳ dị|Rủi ro cao khi đi qua điểm<br>kỳ dị|
|**Ứng dụng Hàn**|Dùng cho lệnh MOVJ (di<br>chuyển nhanh không hàn)|**Bắt buộc cho lệnh MOVL**<br>**(đường hàn thẳng)**|



**Kết luận:** Đối với ứng dụng hàn yêu cầu MOVL, **Ruckig phải được cấu hình chạy trong**
**không gian Cartesian** . Điều này có nghĩa là đầu ra của Ruckig là vị trí Cartesian tiếp theo, và
giá trị này sẽ được đưa vào bộ giải IK của Robotics Library để tính toán góc khớp.

### **2.2. Thiết kế Luồng Dữ liệu Thời gian thực (Hard Real-time Loop)**


Kiến trúc được đề xuất hoạt động trên một chu kỳ điều khiển cứng (ví dụ: 1kHz / 1ms). Dưới
đây là luồng dữ liệu chi tiết từng bước:


**Tầng 1: Thu thập và Tiền xử lý (Start of Cycle)**


1.​ **Đọc Feedback:** Nhận vị trí góc khớp hiện tại ( ) từ Servo Driver qua EtherCAT.
2.​ **Forward Kinematics (FK - Robotics Library):** Sử dụng RL để tính toán vị trí Cartesian


hiện tại ( ) từ .


○​ _Lưu ý:_ Mặc dù Ruckig có thể tự lưu trữ trạng thái nội bộ, việc sử dụng từ
feedback giúp hệ thống bù trừ sai số cơ khí, nhưng có thể gây nhiễu nếu tín hiệu
encoder không sạch. Một phương pháp lai (Hybrid) thường được dùng: Sử dụng


và từ bộ quan sát (Observer), nhưng dùng làm đầu
vào vị trí để tránh rung động do lượng tử hóa encoder.


**Tầng 2: Tạo Quỹ đạo (Ruckig OTG)**


3.​ **Cập nhật Mục tiêu (Application Layer):** Nếu đang thực hiện Seam Tracking, cập nhật

Target Position mới dựa trên dữ liệu cảm biến (chi tiết ở Chương 4).
4.​ **Thực thi Ruckig Update:**


○​ **Input:** Trạng thái hiện tại ( ), Trạng thái mục tiêu (


), Giới hạn ( ).
○​ **Function:** ruckig.update(input, output).


○​ **Output:** Trạng thái Cartesian tiếp theo cho chu kỳ tới ( ).


**Tầng 3: Giải Động học Nghịch (RL Core Math)**


5.​ **Inverse Kinematics (IK - Robotics Library):**

○​ Sử dụng bộ giải IK (thường là rl::mdl::NloptInverseKinematics hoặc thuật toán

Newton-Raphson lặp cho tốc độ cao).


○​ **Input:** (Tọa độ mong muốn từ Ruckig).


○​ **Seed:** (Góc khớp hiện tại làm điểm khởi đầu để đảm bảo IK hội tụ về cấu
hình gần nhất).


○​ **Output:** (Góc khớp lệnh).


**Tầng 4: Hậu xử lý và An toàn**


6.​ **Kiểm tra Giới hạn:** So sánh với giới hạn mềm của khớp. Tính toán vận tốc khớp


yêu cầu . Nếu vượt quá giới hạn vật lý (do đi qua điểm kỳ
dị), kích hoạt dừng khẩn cấp hoặc chế độ giảm tốc (Scaling).


7.​ **Gửi Lệnh:** Gửi xuống Servo Driver.
8.​ **Lưu Trạng thái:** Sao chép output của Ruckig vào input cho chu kỳ tiếp theo

(output.pass_to_input(input)).

## **3. Chuyển động MOVL và Đồng bộ hóa** **(Synchronization)**


Yêu cầu "MOVL" (Move Linear) trong robot hàn đòi hỏi đầu công tác di chuyển theo một
đường thẳng tuyệt đối trong không gian 3D, đồng thời duy trì hướng mỏ hàn thay đổi mượt
mà. Trong Ruckig, việc này phụ thuộc hoàn toàn vào cơ chế **Đồng bộ hóa (Synchronization)** .

### **3.1. Phân tích các Chế độ Đồng bộ hóa**


Ruckig Community cung cấp các chế độ đồng bộ hóa sau [5] :






|Chế độ (Enum)|Cơ chế hoạt động|Kết quả Quỹ đạo<br>(Cartesian)|
|---|---|---|
|Synchronization::None|Mỗi trục (X, Y, Z...) tính toán<br>thời gian tối ưu riêng biệt.<br>Trục nào xong trước sẽ<br>dừng lại đợi.|**Đường gấp khúc/Cong.** <br>TCP sẽ di chuyển ziczac<br>hoặc theo đường cong<br>ngẫu nhiên.**Không dùng**<br>**cho hàn.**|
|Synchronization::Time|Tính toán thời gian của trục<br>chậm nhất (<br>), sau đó<br>kéo dài thời gian của các<br>trục còn lại để cùng kết<br>thúc tại<br>.|**Đường cong.** Mặc dù bắt<br>đầu và kết thúc cùng lúc,<br>nhưng biên dạng vận tốc<br>khác nhau dẫn đến quỹ đạo<br>không thẳng.|
|Synchronization::Phase|Không chỉ đồng bộ thời<br>gian đích, mà còn đồng bộ<br>**pha chuyển động**. Các<br>trục được tỷ lệ hóa (scaled)<br>sao cho tại mọi thời điểm<br>, tỷ lệ quãng đường đi được<br>là hằng số.|**Đường thẳng (Straight**<br>**Line).** Đây là chìa khóa của<br>MOVL. Đảm bảo vector vận<br>tốc luôn hướng về đích.|



**Khuyến nghị Kỹ thuật:** Để thực hiện MOVL, bạn **bắt buộc** phải cấu hình Ruckig sử dụng
Synchronization::Phase.


C++

### **3.2. Tại sao "Phase Synchronization" tạo ra đường thẳng?**


Về mặt toán học, nếu ta có chuyển động từ đến . Một điểm nằm trên đoạn


thẳng nối hai điểm này nếu và chỉ nếu: Trong đó là
một hàm vô hướng đi từ 0 đến 1. Chế độ Phase của Ruckig đảm bảo rằng tất cả các bậc tự do


(X, Y, Z) đều tuân theo cùng một biên dạng thời gian (profile 1-DOF được tính cho trục
hạn chế nhất). Do đó, phương trình đường thẳng được thỏa mãn. [4 ]

### **3.3. Xử lý Chuyển động Xoay (Orientation)**


Với robot 6-DOF, ngoài X, Y, Z, ta còn phải nội suy hướng (Roll, Pitch, Yaw).


●​ Ruckig xử lý hướng như các biến số thực (double).
●​ **Vấn đề:** Euler Angles có thể gặp vấn đề về tính không liên tục (nhảy từ 179 độ sang -179

độ).
●​ **Giải pháp:** Trong ứng dụng hàn (thường mỏ hàn chúi xuống), việc sử dụng Euler Angles

(ZYX hoặc XYZ) thường đủ an toàn và trực quan. Tuy nhiên, cần một bước "Unwind" (tháo
gỡ) góc trước khi đưa vào Ruckig. Ví dụ: Nếu góc hiện tại là 10 độ và đích là 350 độ,
Ruckig sẽ hiểu là đi quãng đường +340 độ. Ta cần chuyển đổi đích thành -10 độ để
quãng đường là -20 độ.
●​ **Đồng bộ hóa Xoay:** Khi dùng Phase cho cả 6 trục, việc xoay mỏ hàn cũng sẽ được phân

bố đều dọc theo chiều dài đường hàn. Điều này rất lý tưởng cho kỹ thuật hàn leo hoặc
hàn góc, nơi góc mỏ hàn cần thay đổi dần dần từ đầu đến cuối đường hàn.

## **4. Giải pháp Seam Tracking với "Arbitrary Target** **States"**


Yêu cầu tích hợp Seam Tracking là thách thức lớn nhất khi dùng phiên bản Community vì thiếu
Interface Tracking chuyên dụng. [4] Tuy nhiên, bản chất của Ruckig là thuật toán
"State-to-State" cực nhanh, cho phép ta cập nhật đích đến (Target) trong mỗi chu kỳ điều
khiển.

### **4.1. Nguyên lý "Moving Target" (Mục tiêu Di động)**


Thay vì coi đường hàn là một tập hợp các điểm cố định, hãy coi nó là một mục tiêu đang di
chuyển.


Trong mỗi chu kỳ 1ms:


1.​ **Đọc Cảm biến:** Cảm biến Laser Vision trả về độ lệch ngang ( ) và độ lệch cao ( )
so với khung tọa độ mỏ hàn.
2.​ **Biến đổi Tọa độ:** Chuyển đổi vector lệch này từ khung cảm biến (Sensor Frame) sang

khung thế giới (World Frame). ​


3.​ **Cộng dồn (Integration):** Không chỉ cộng độ lệch vào vị trí hiện tại, ta phải cộng nó vào

**Vị trí Mục tiêu (Target Position)** . ​


4.​ **Cập nhật Ruckig:** Gán giá trị mới này vào input.target_position và gọi ruckig.update().


Ruckig sẽ tính toán lại quỹ đạo từ trạng thái hiện tại (đang di chuyển với vận tốc ) để đến
mục tiêu mới. Do Ruckig hỗ trợ vận tốc đầu khác 0, chuyển động chuyển tiếp sẽ rất mượt mà,
không bị giật cục. [2 ]

### **4.2. Xử lý "Arbitrary Target States" (Trạng thái Mục tiêu Bất kỳ)**


Thuật ngữ "Arbitrary Target States" [2] trong tài liệu Ruckig ám chỉ khả năng thay đổi đích đến
bất kỳ lúc nào. Điều này cực kỳ quan trọng cho Seam Tracking vì:


●​ Nếu cảm biến phát hiện đường hàn cong đột ngột, đích đến thay đổi lớn.
●​ Ruckig sẽ tự động điều chỉnh gia tốc (trong giới hạn Jerk) để lái robot về đích mới nhanh

nhất có thể.


**Lưu ý quan trọng về vận tốc Tracking:**


Khi tracking, robot không bao giờ thực sự "đến" đích vì đích luôn bị đẩy ra xa (dọc theo đường
hàn). Để duy trì tốc độ hàn (Travel Speed) ổn định, input.target_velocity cần được thiết lập.


●​ Nếu tracking điểm cố định: .


●​ Nếu tracking đường hàn đang chạy: nên được đặt là vector vận tốc hàn danh
định. Điều này giúp Ruckig dự đoán chuyển động (Feedforward), giảm thiểu sai số bám
(Lag error).

### **4.3. Bộ lọc và Ổn định (Signal Smoothing)**


Dữ liệu từ cảm biến laser thường có nhiễu (noise). Nếu đưa trực tiếp nhiễu này vào
target_position của Ruckig, robot sẽ rung lắc vì Ruckig sẽ cố gắng phản ứng với từng gai nhiễu
đó (do tính chất "Real-time reaction" [2] ).


**Giải pháp:** Cần một bộ lọc trung gian.


1.​ **Bộ đệm vòng (Ring Buffer):** Lấy trung bình cộng 10-20 mẫu dữ liệu cảm biến gần nhất.


2.​ **Giới hạn biên độ (Saturation):** Chỉ cho phép sửa đổi mục tiêu tối đa mm mỗi chu kỳ
để tránh các cú nhảy bất thường do lỗi đọc cảm biến.

## **5. Hiện thực hóa bằng C++ và Robotics Library (RL)**


Dưới đây là mẫu code C++ minh họa kiến trúc tích hợp. Code giả định bạn đã có môi trường
RL và Ruckig.

### **5.1. Cấu trúc dữ liệu và Khởi tạo**


C++


​

​

​
using namespace ruckig;​
​

​

​

​

​


​

​

​

​

​

​

​

​

​

​
// Nếu có lệch, cộng dồn vào Target Position HIỆN TẠI​


// for(int i=0; i<3; ++i) input.target_position[i] += sensor_offset[i];​
​

​
if (result == Result::Working |​
​
| result == Result::Finished) {​
​

​
// --- BƯỚC C: GIẢI IK (Robotics Library) ---​
​

​

​

​

​

​

​


}​
​

​

​

### **5.2. Giải thích Chi tiết Code**


1.​ **Ruckig<DOFs> otg(0.001);** : Khởi tạo instance Ruckig với chu kỳ 1ms. Tham số này cực

kỳ quan trọng để tích phân vị trí chính xác.
2.​ **input.synchronization = Synchronization::Phase;** : Dòng lệnh quyết định tính chất

"thẳng" của đường hàn. Nếu bỏ qua, mỏ hàn sẽ đi đường cong.
3.​ **Vòng lặp while** : Mô phỏng Thread thời gian thực. Trong hệ thống thực (như Xenomai hay

RT-Linux), toàn bộ khối lệnh trong while phải thực thi xong trong <1ms.

○​ Ruckig update() thường tốn khoảng 20-50µs. [2 ]

○​ RL calculateInversePosition() (Newton-Raphson) có thể tốn từ 100-300µs tùy độ hội

tụ. Cần chú ý monitor thời gian này.
4.​ **output.pass_to_input(input);** : Đây là cơ chế "State-to-State". Nó copy trạng thái động


học ( ) vừa tính được làm điểm khởi đầu cho chu kỳ sau, đảm bảo tính liên tục
C2 (gia tốc liên tục) cho đường S-Curve.

## **6. Các vấn đề Kỹ thuật Nâng cao**
### **6.1. Xử lý Điểm Kỳ dị (Singularities)**


Khi chạy Ruckig trong không gian Cartesian, rủi ro lớn nhất là quỹ đạo đường thẳng đi ngang
qua điểm kỳ dị của robot (ví dụ: cổ tay thẳng hàng với cánh tay).


●​ **Triệu chứng:** Tại gần điểm kỳ dị, để duy trì vận tốc Cartesian không đổi, vận tốc khớp (


) sẽ tăng vọt tới vô cùng.
●​ **Giải pháp trong Code:** ​

Sau bước giải IK, cần tính vận tốc khớp dự kiến: ​


​


Nếu, bạn phải can thiệp. Ruckig Community không hỗ trợ
ràng buộc vận tốc khớp tự động khi chạy ở chế độ Cartesian. Bạn phải thực hiện "Velocity
Scaling": giảm input.target_velocity hoặc tạm thời giảm max_velocity Cartesian và tính lại
Ruckig.

### **6.2. Giới hạn Thời gian quỹ đạo (The 7e3 Limit)**


Tài liệu Ruckig đề cập giới hạn thời gian quỹ đạo là (khoảng 2 giờ). [1 ]


●​ **Tác động:** Với các đường hàn thông thường (vài phút), đây không phải vấn đề.
●​ **Xử lý:** Nếu robot vận hành liên tục (ví dụ: tracking băng chuyền vô tận), biến thời gian nội

bộ có thể tràn. Tuy nhiên, Ruckig reset thời gian mỗi khi đạt đích (Result::Finished). Với kỹ
thuật "Arbitrary Target States", nếu đích thay đổi liên tục và robot không bao giờ "dừng
hẳn", cần theo dõi cẩn thận. Trong thực tế hàn, robot luôn có các điểm dừng (tắt hồ
quang), nên bộ đếm sẽ được reset tự nhiên.

### **6.3. Giới hạn của Robotics Library (RL)**


RL là một thư viện mạnh nhưng thiên về tính toán "offline" hoặc tĩnh. Hàm
calculateInversePosition sử dụng phương pháp lặp.


●​ **Tối ưu hóa:** Cần cài đặt số lần lặp tối đa (Max Iterations) và dung sai (Tolerance) phù hợp

cho bài toán thời gian thực (ví dụ: 1e-4 mét, 50 iterations). Nếu quá chính xác, IK sẽ tốn
quá nhiều thời gian (>1ms), làm vỡ chu kỳ điều khiển (Cycle miss).

## **7. Kết luận**


Việc tích hợp Ruckig Community Version vào bộ điều khiển robot hàn 6-DOF sử dụng Robotics
Library là hoàn toàn khả thi và mang lại hiệu suất vượt trội so với các bộ lập kế hoạch truyền
thống. Chìa khóa thành công nằm ở 3 điểm:


1.​ **Kiến trúc Cartesian OTG:** Cho phép kiểm soát trực tiếp hình dáng đường hàn (MOVL) và

tốc độ hàn.
2.​ **Phase Synchronization:** Đảm bảo độ thẳng tuyệt đối của đường hàn trong không gian

3D.
3.​ **Cơ chế Arbitrary Target Updates:** Vượt qua giới hạn không có API Tracking của bản

Community, cho phép phản hồi thời gian thực với cảm biến Seam Tracking.


Mô hình này không chỉ đáp ứng yêu cầu về chuyển động mượt mà (S-Curve) giúp nâng cao
chất lượng mối hàn mà còn mở ra khả năng thích ứng linh hoạt với sai số phôi thông qua cảm
biến, đưa bộ điều khiển robot lên tầm cao mới về công nghệ.


**Works cited**



1.​ Background Information - Ruckig, accessed February 1, 2026,



[htps://docs.ruckig.com/background.html](https://docs.ruckig.com/background.html)
2.​ Ruckig - Motion Generation for Robots and Machines, accessed February 1, 2026,



[htps://ruckig.com/](https://ruckig.com/)
3.​ ruckig - ROS Repository Overview, accessed February 1, 2026,



[htps://index.ros.org/r/ruckig/](https://index.ros.org/r/ruckig/)
4.​ [Tutorial - Ruckig, accessed February 1, 2026, htps://docs.ruckig.com/tutorial.html](https://docs.ruckig.com/tutorial.html)
5.​ ruckig Namespace Reference, accessed February 1, 2026,



[htps://docs.ruckig.com/namespaceruckig.html](https://docs.ruckig.com/namespaceruckig.html)
6.​ pantor/ruckig: Motion Generation for Robots and Machines. Real-time.



Jerk-constrained. Time-optimal. - GitHub, accessed February 1, 2026,
[htps://github.com/pantor/ruckig](https://github.com/pantor/ruckig)
7.​ [2105.04830] Jerk-limited Real-time Trajectory Generation with Arbitrary Target



States, accessed February 1, 2026, [htps://arxiv.org/abs/2105.04830](https://arxiv.org/abs/2105.04830)


