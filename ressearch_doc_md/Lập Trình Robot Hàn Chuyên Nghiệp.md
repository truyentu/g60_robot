# **Báo cáo Nghiên cứu Chuyên sâu: Phát** **triển Module WeldPatternGenerator cho** **Hệ thống Điều khiển Robot Hàn 6 Trục**
## **1. Tổng quan và Kiến trúc Hệ thống Điều khiển**
### **1.1. Giới thiệu**

Trong lĩnh vực chế tạo robot công nghiệp, đặc biệt là robot hàn hồ quang (Arc Welding
Robot), phần mềm điều khiển đóng vai trò cốt lõi trong việc định hình chất lượng sản phẩm.
Trong khi các bộ nội suy chuyển động cơ bản (Linear/Joint Interpolation) giải quyết vấn đề đưa
mỏ hàn từ điểm A đến điểm B, thì module tạo mẫu đường hàn (WeldPatternGenerator) mới
chính là thành phần quyết định tính "nghệ thuật" và độ bền kết cấu của mối hàn. Yêu cầu phát
triển một module có khả năng xử lý Weaving (dao động), Dwell Time (thời gian dừng), Stitch
Welding (hàn đính), và Multi-pass (hàn nhiều lớp) đòi hỏi sự am hiểu sâu sắc về toán học
không gian, động học robot và vật lý hồ quang. [1 ]


Báo cáo này cung cấp một phân tích toàn diện, đi sâu vào các công thức toán học, logic điều
khiển và giả mã (pseudo-code) cần thiết để triển khai các kỹ thuật này bằng ngôn ngữ C++.
Mục tiêu là cung cấp một tài liệu tham khảo kỹ thuật chi tiết, hỗ trợ việc chuyển đổi các lý
thuyết hàn thành các thuật toán điều khiển thời gian thực chính xác.

### **1.2. Vai trò của WeldPatternGenerator trong Kiến trúc Điều khiển**


Trong kiến trúc phần mềm điều khiển robot tiêu chuẩn, WeldPatternGenerator hoạt động như
một lớp trung gian (middleware) giữa bộ lập kế hoạch quỹ đạo (Trajectory Planner) và bộ giải
động học ngược (Inverse Kinematics Solver).


●​ **Đầu vào:** Quỹ đạo danh nghĩa (Nominal Path) được xác định bởi các điểm dạy (Teaching


Points) bao gồm vị trí ( ) và hướng ( hoặc Quaternion), cùng
với các tham số công nghệ hàn (biên độ lắc, tần số, thời gian dừng).
●​ **Xử lý:** Module này tính toán các vector bù trừ (offset vectors) thời gian thực dựa trên hàm

sóng (waveform function) và hệ tọa độ cục bộ tại đầu mỏ hàn (TCP - Tool Center Point).


●​ **Đầu ra:** Tọa độ TCP biến thiên theo thời gian ( ) được gửi đến bộ giải động học


ngược để tính toán các góc khớp ( ). [1 ]


Sự tách biệt giữa "chuyển động mang" (carrier motion - di chuyển dọc theo đường hàn) và
"chuyển động tạo hình" (pattern motion - dao động ngang) là nguyên tắc cơ bản để thiết kế
module này. Điều này cho phép người vận hành thay đổi kiểu dao động mà không cần dạy lại


đường đi cơ bản.

## **2. Cơ sở Toán học: Hệ Tọa độ và Khung Dệt (Weaving** **Frame)**


Để thực hiện bất kỳ kỹ thuật Weaving nào, trước hết ta phải xác định được một hệ tọa độ cục
bộ di chuyển dọc theo đường hàn. Hệ tọa độ này xác định phương "ngang" (để lắc sang hai
bên) và phương "dọc" (hướng di chuyển), cũng như phương "pháp tuyến" (hướng vào bề mặt
vật liệu).

### **2.1. Vector Tiếp tuyến, Pháp tuyến và Trùng pháp tuyến**


Giả sử quỹ đạo hàn được tham số hóa bởi độ dài cung, vị trí tâm tại là . Ta cần xác


định bộ ba vector trực giao tại mỗi điểm trên quỹ đạo. Tuy nhiên, việc sử dụng
khung Frenet-Serret tiêu chuẩn (dựa trên đạo hàm bậc hai của đường cong) thường gặp vấn
đề tại các đoạn đường thẳng (nơi độ cong bằng 0, dẫn đến vector pháp tuyến không xác
định). [4 ]


Trong ứng dụng robot hàn, phương pháp tiếp cận mạnh mẽ hơn là sử dụng hướng của mỏ hàn
để định nghĩa mặt phẳng dao động.


**Định nghĩa các Vector:**


1.​ **Vector Tiếp tuyến (** **):** Hướng di chuyển của mỏ hàn. ​


​
Trong điều khiển thời gian thực, đây là vector vận tốc tuyến tính chuẩn hóa:


.


2.​ **Vector Hướng Tiếp cận (** **):** Hướng trục Z của mỏ hàn (Tool Z-axis), hướng từ mặt bích
robot đến đầu dây hàn. Đây là thông số đã biết từ động học thuận.


3.​ **Vector Dao động (** **- Weave Direction):** Đây là vector quan trọng nhất, xác định
phương ngang để mỏ hàn lắc sang trái/phải. Vector này phải vuông góc với hướng di


chuyển và nằm trong mặt phẳng mong muốn. Công thức tổng quát nhất để tính
(tương ứng với vector Trùng pháp tuyến hoặc Binormal trong ngữ cảnh hàn) là tích có
hướng của Vector Tiếp tuyến và Vector Hướng Tiếp cận [1] : ​


​


_Lưu ý:_ Nếu song song với (trường hợp hàn chúi xuống thẳng đứng), phép tính này
sẽ sinh ra điểm kỳ dị (singularity). Trong thực tế, mỏ hàn luôn có một góc nghiêng (travel
angle) hoặc góc làm việc (work angle), nên tích có hướng này thường hợp lệ. Nếu xảy ra
kỳ dị, cần sử dụng vector tham chiếu bổ sung (ví dụ: trục Z của hệ tọa độ World).


4.​ **Vector Pháp tuyến Bề mặt Ảo (** **):** ​


​
Vector này giúp xác định chiều cao (nếu cần lắc theo phương đứng, ví dụ kiểu Figure-8).

### **2.2. Ma trận Biến đổi Homogeneous**


Vị trí thực tế của TCP khi có Weaving ( ) được tính bằng vị trí trên đường dẫn gốc ( )


cộng với độ lệch cục bộ ( ) đã được quay về hệ tọa độ gốc (Base Frame). [1 ]


Trong đó $ =$ là ma trận quay được tạo bởi các vector cơ sở đã tính ở trên. là hàm dao
động chuẩn hóa.

## **3. Kỹ thuật Weaving (Hàn Lắc): Phân tích và Triển khai** **C++**


Weaving là kỹ thuật dao động mỏ hàn ngang qua rãnh hàn để mở rộng bể hàn, tăng khả năng
điền đầy và cải thiện sự hòa trộn kim loại. [2] Về mặt toán học, đây là việc tạo ra một tín hiệu


tuần hoàn phụ thuộc vào pha .

### **3.1. Quản lý Pha (Phase) và Tần số**


Pha dao động biến thiên theo thời gian . Trong hệ thống rời rạc (digital controller), pha


được cập nhật sau mỗi chu kỳ lấy mẫu (ví dụ: 0.001s).


**Công thức cập nhật pha:**


Trong đó là tần số lắc (Hz). Pha thường được chuẩn hóa trong khoảng hoặc $$
tùy quy ước lập trình.

### **3.2. Các Kiểu Dao động (Patterns) và Công thức Toán học**


Dưới đây là chi tiết toán học cho các kiểu Weaving phổ biến được yêu cầu [1] :


**3.2.1. Sine Wave (Hình Sin)**


Đây là dạng cơ bản nhất, tạo ra chuyển động mượt mà, giảm rung động cơ khí cho robot.


●​ **Ứng dụng:** Hàn phủ (cover pass), yêu cầu bề mặt đẹp.
●​ **Công thức:** ​


●​ **Đặc điểm nhiệt:** Nhiệt lượng tập trung nhiều hơn ở biên (nơi vận tốc ngang tiến tới 0) và

ít hơn ở giữa (nơi vận tốc ngang cực đại). Điều này tự nhiên giúp chống cháy cạnh
(undercut). [1 ]


**3.2.2. Triangle Wave (Hình Tam giác)**


Chuyển động tuyến tính giữa hai biên.

●​ **Ứng dụng:** Hàn góc (Fillet), hàn rãnh chữ V, yêu cầu điền đầy đều và ngấu sâu tại góc. [6 ]

●​ **Công thức:** ​


Hàm tam giác chuẩn hóa biên độ với chu kỳ : ​
$$ \delta_{tri}(\phi) = \begin{cases} ​
\frac{2}{\pi}\phi & 0 \le \phi < \frac{\pi}{2} \ ​
2 - \frac{2}{\pi}\phi & \frac{\pi}{2} \le \phi < \frac{3\pi}{2} \ ​
\frac{2}{\pi}\phi - 4 & \frac{3\pi}{2} \le \phi < 2\pi ​
\end{cases} $$


●​ **Vấn đề Gia tốc:** Tại các đỉnh ( ), vận tốc đổi chiều tức thời gây ra gia tốc vô hạn về lý
thuyết. Trong lập trình C++, cần áp dụng bộ lọc làm mượt (Smoothing/Corner Rounding)



​


tại các đỉnh này để bảo vệ hộp số robot. [8 ]


**3.2.3. Trapezoidal Wave (Hình Thang)**


Tương tự hình tam giác nhưng có đoạn nằm ngang tại biên.


●​ **Ứng dụng:** Tăng cường nung chảy thành bên (sidewall fusion) cho vật liệu dày.
●​ **Công thức:** Được xác định bởi tham số Dwell Time (xem phần 4). Về mặt hình học, nó


bão hòa tại .


**3.2.4. Circular Wave (Hình Tròn)**


Mỏ hàn vẽ các vòng tròn nhỏ dọc theo đường hàn.


●​ **Ứng dụng:** Hàn đắp bề mặt (surfacing/cladding), kiểm soát vũng hàn lỏng khi hàn trần

(overhead). [2 ]

●​ **Công thức:** ​


Khác với Sine/Triangle chỉ dao động trên trục, Circular dao động trên cả trục và


trục (tiếp tuyến). ​


_Lưu ý:_ Việc thêm offset vào trục làm thay đổi vận tốc di chuyển tức thời của mỏ hàn so
với phôi (khi quay ngược chiều di chuyển, mỏ hàn đi chậm lại; khi quay cùng chiều, nó đi
nhanh hơn).


**3.2.5. Figure-8 (Hình số 8)**


●​ **Ứng dụng:** Hàn leo (Vertical Up), vật liệu rất dày. Giúp vũng hàn nguội bớt khi mỏ hàn đảo

chiều, ngăn kim loại lỏng chảy xệ. [9 ]

●​ **Công thức (Lemniscate of Gerono):** ​


Trong đó là biên độ dọc (thường nhỏ hơn biên độ ngang ).


**3.2.6. L-Type Weaving (Kiểu chữ L)**


Đây là kỹ thuật đặc biệt cho hàn góc (Fillet Weld), nơi biên dạng dao động không đối xứng qua



​


​


​


​


tâm mà bám theo hai cạnh của góc vuông (thành đứng và sàn). [8 ]


●​ **Logic:** ​

Thay vì dao động quanh một điểm trung tâm, L-Type di chuyển từ: ​


Gốc (Root) Lên thành đứng (Vertical Wall) Về Gốc Ra sàn ngang (Horizontal


Floor) Về Gốc.


●​ **Vector cơ sở:** Cần xác định hai vector riêng biệt: (thường là trục Z của Robot hoặc


vector dạy trước) và .
●​ **Thuật toán:**


○​ Nếu : Di chuyển dọc .


○​ Nếu : Di chuyển dọc .

### **3.3. Giả mã C++ cho Module Weaving**


Dưới đây là cấu trúc class và hàm tính toán offset cốt lõi:


C++


​

​

​
public:​


​

​

​

​

​

​

​


## **4. Logic Điều khiển Dwell Time (Thời gian dừng)**

Dwell Time là khoảng thời gian mỏ hàn dừng lại (hoặc di chuyển rất chậm) tại hai biên của
biên độ lắc. Đây là yếu tố quyết định để tránh khuyết tật "undercut" (cháy cạnh) và đảm bảo
độ ngấu thành bên. [1 ]

### **4.1. Cơ chế Vật lý**


Khi hàn lắc, mỏ hàn di chuyển qua trung tâm rãnh hàn với tốc độ cao nhất (đối với Sine) hoặc
bằng nhau (Triangle). Tuy nhiên, tại thành bên (sidewall), nhiệt lượng bị tản nhanh vào phôi.
Nếu mỏ hàn quay đầu ngay lập tức, nhiệt lượng cung cấp không đủ để làm nóng chảy thành
bên, gây ra khuyết tật thiếu ngấu (lack of fusion). Việc "Dwell" giúp cung cấp đủ nhiệt lượng
tại biên.

### **4.2. Logic Trạng thái (State Machine) cho Dwell**


Trong phần mềm điều khiển, Dwell không chỉ đơn giản là một hàm toán học mà là một máy
trạng thái (State Machine) can thiệp vào bộ cập nhật pha.


**Thuật toán Dwell:**


Thay vì chỉ tính, hệ thống cần kiểm tra xem vị trí hiện tại có đạt tới biên độ cực đại hay
không.


1.​ **Trạng thái MOVE:** Tăng pha . Tính toán vị trí. Nếu, chuyển
sang trạng thái DWELL.
2.​ **Trạng thái DWELL:**


○​ Ngưng tăng pha (giữ nguyên tại hoặc ).
○​ Kích hoạt bộ đếm thời gian (Timer).


○​ Tùy chọn hành vi chuyển động dọc ( ):
■​ **Weave Stop:** Chỉ dừng dao động ngang, robot vẫn tiến về phía trước (tạo ra

đường hàn hình thang trên quỹ đạo).
■​ **Robot Stop:** Dừng toàn bộ chuyển động robot (TCP đứng yên tuyệt đối). Đây là

chế độ phổ biến cho hàn dày. [8 ]


○​ Khi Timer DwellTime cài đặt Chuyển về trạng thái MOVE, tiếp tục tăng .


### **4.3. Giả mã Logic Dwell (Tích hợp vào CalculateOffset)**

C++

## **5. Kỹ thuật Stitch Welding (Hàn Đính) và Logic Crater** **Fill**


Stitch Welding (hay Intermittent Welding) là kỹ thuật tạo ra các đoạn hàn ngắn ngắt quãng,
thường dùng để giảm biến dạng nhiệt trên vật liệu mỏng hoặc gá lắp tạm thời. [14 ]

### **5.1. Sơ đồ Thời gian (Timing Diagram) và Máy Trạng thái**


Stitch Welding yêu cầu phối hợp chặt chẽ giữa chuyển động robot và tín hiệu kích hoạt máy
hàn (Arc On/Off).


**Chu trình chuẩn:**


1.​ **Approach:** Robot di chuyển nhanh đến điểm bắt đầu đoạn đính.
2.​ **Arc Start:** Gửi tín hiệu ArcOn. Robot đợi tín hiệu phản hồi ArcEstablished từ nguồn hàn.
3.​ **Weld:** Di chuyển quãng đường StitchLength với vận tốc hàn.
4.​ **Crater Fill (Lấp rãnh):**

○​ Khi đến cuối đoạn đính, robot không tắt hồ quang ngay.
○​ Robot giảm tốc độ hoặc dừng hẳn.
○​ Gửi tín hiệu chuyển nguồn hàn sang chế độ "Crater" (Dòng điện/Điện áp thấp hơn).
○​ Duy trì trong thời gian CraterTime.
5.​ **Arc End:** Gửi tín hiệu ArcOff.
6.​ **Burnback:** Đợi dây hàn cháy ngược một chút để không dính vào phôi.
7.​ **Air Move:** Di chuyển nhanh (không hàn) quãng đường Pitch (hoặc Gap) đến đoạn tiếp

theo.

### **5.2. Logic Crater Fill**


Tại sao cần Crater Fill? Khi hồ quang tắt đột ngột, vũng hàn kim loại lỏng co lại khi nguội, tạo
ra một vết lõm (crater) ở cuối đường hàn. Đây là điểm yếu chịu lực và dễ nứt. [14 ]


**Các chiến thuật điều khiển:**


●​ **Time-based:** Dừng robot, giảm dòng hàn, giữ trong giây.
●​ **Back-step:** Tại điểm cuối, robot đi lùi lại một khoảng nhỏ (ví dụ 5mm) vào trong vũng hàn

rồi mới tắt hồ quang. [18 ]

●​ **Ramp-down:** Giảm dần tốc độ cấp dây (Wire Feed Speed) trong khi robot vẫn di chuyển

chậm lại.

### **5.3. Bảng Dữ liệu Điều khiển Stitch**


|Tham số|Đơn vị|Mô tả|
|---|---|---|
|Stitch Length|mm|Chiều dài đoạn hàn|
|Stitch Pitch|mm|Khoảng cách giữa các điểm<br>bắt đầu (hoặc Gap Length)|
|Crater Time|sec|Thời gian điền đầy rãnh<br>cuối|


|Crater Volts/Amps|V/A|Thông số hàn cho giai đoạn<br>lấp rãnh|
|---|---|---|
|Burnback Time|sec|Thời gian chờ dây cháy<br>ngược|

## **6. Chiến lược Multi-pass Offset (Hàn Nhiều Lớp)**

Đối với vật liệu dày (thường > 10mm), một đường hàn đơn không thể điền đầy rãnh. Cần thực


hiện hàn nhiều lớp (Multi-pass). Thách thức toán học ở đây là tính toán vị trí offset
cho từng đường hàn (pass) sao cho chúng xếp chồng lên nhau khít khao, không để lại lỗ
hổng. [19 ]

### **6.1. Nguyên lý Hình học và Quy tắc Xếp chồng**


Có hai mô hình rãnh hàn chính: V-Groove và Fillet. Nguyên tắc cơ bản là **Quy tắc 50%**
**Overlap** : Đường hàn sau nên đè lên đường hàn trước khoảng 50% bề rộng để đảm bảo bề
mặt lớp hàn phẳng. [22 ]


Diện tích tiết diện ngang của mỗi đường hàn ( ) có thể ước tính xấp xỉ bằng công thức


Parabol hoặc hình bán nguyệt dựa trên tốc độ cấp dây ( ) và tốc độ di chuyển ( ):


Trong đó là diện tích tiết diện dây hàn, là hiệu suất đắp.

### **6.2. Thuật toán Tính toán Offset cho V-Groove**


Giả sử rãnh chữ V có góc, khe hở đáy . Ta cần xếp các đường hàn vào các lớp (Layers).


**Công thức tính toán:**


1.​ **Chiều cao lớp (** **):** Thường bằng độ dày hiệu dụng của một đường hàn (ví dụ 2-3mm).


2.​ **Bề rộng rãnh tại lớp** **(** **):** ​


3.​ **Số đường hàn trong lớp** **(** **):** ​


Dựa trên bề rộng đường hàn đơn ( ) và độ chồng lấn ( ). ​


4.​ **Offset ngang (** **) cho đường thứ trong lớp** **:** ​


​
(Tính từ tâm rãnh).


5.​ **Offset đứng (** **):** ​

### **6.3. Giả mã C++ cho Multi-pass Generator**


C++


​

​

​

​
// Tính số đường hàn cần thiết (dựa trên overlap)​


​

​

### **6.4. Các yếu tố ảnh hưởng bậc hai**


●​ **Biến dạng nhiệt:** Khi hàn nhiều lớp, phôi sẽ bị co rút (transverse shrinkage), làm rãnh


hẹp lại. [24] Các phần mềm cao cấp thường có tham số bù co ngót, giảm dần theo số
lớp thực tế đo được hoặc dự đoán.
●​ **Thứ tự hàn:** Để giảm biến dạng, thứ tự hàn trong một lớp thường là so le hoặc từ ngoài

vào trong, thay vì chỉ đơn thuần từ trái sang phải.

## **7. Kết luận và Khuyến nghị Triển khai**


Việc xây dựng module WeldPatternGenerator cho robot 6 trục là một bài toán phức tạp đòi hỏi
sự kết hợp giữa toán học hình học (cho Weaving/Multi-pass) và logic máy trạng thái thời gian
thực (cho Dwell/Stitch).


**Các điểm chính cần lưu ý khi lập trình C++:**


1.​ **Tính Modularity:** Tách biệt lớp tính toán offset (PatternGenerator) khỏi lớp nội suy


chuyển động (Interpolator). Interpolator chỉ nên nhận đầu vào là vector offset tại
mỗi chu kỳ 1ms.
2.​ **Xử lý ngoại lệ:** Chú ý các điểm kỳ dị khi tính toán vector Binormal (khi mỏ hàn song song

với đường đi). Luôn có vector dự phòng (fallback vector).
3.​ **Làm mượt (Smoothing):** Các dạng sóng tam giác hoặc hình thang lý thuyết có giật cục


về gia tốc. Cần áp dụng bộ lọc thông thấp (Low-pass filter) hoặc Spline để làm mượt quỹ
đạo trước khi gửi xuống servo.
4.​ **Tối ưu hóa:** Các hàm lượng giác (sin, cos, atan2) tốn kém CPU. Nên sử dụng bảng tra

(Look-up Table) hoặc xấp xỉ chuỗi Taylor nếu vi xử lý điều khiển có tài nguyên hạn chế,
tuy nhiên với các controller hiện đại (ARM Cortex-M7 hoặc x86), việc tính toán trực tiếp
thường vẫn khả thi.


Bằng cách tuân thủ các công thức và kiến trúc trên, bạn có thể xây dựng một bộ điều khiển
hàn robot chuyên nghiệp, có khả năng cạnh tranh với các giải pháp thương mại từ Fanuc hay
Yaskawa.


**Works cited**



1.​ An Algorithm for the Welding Torch Weaving Control of Arc Welding Robot 


Semantic Scholar, accessed February 1, 2026,
[htps://pdfs.semanticscholar.org/91d2/5b5cef87617741edce7b7c954f1141293ef7.p](https://pdfs.semanticscholar.org/91d2/5b5cef87617741edce7b7c954f1141293ef7.pdf)
[df](https://pdfs.semanticscholar.org/91d2/5b5cef87617741edce7b7c954f1141293ef7.pdf)
2.​ Weaving in welding | A comprehensive guide to weave patterns - Smooth



Robotics, accessed February 1, 2026,
[htps://smooth-robotics.com/weaving-in-welding/](https://smooth-robotics.com/weaving-in-welding/)
3.​ Towards a Uniform Welding Quality: A Novel Weaving Welding ..., accessed



February 1, 2026, [htps://pmc.ncbi.nlm.nih.gov/articles/PMC9181624/](https://pmc.ncbi.nlm.nih.gov/articles/PMC9181624/)
4.​ Universal Path-Following of Wheeled Mobile Robots: A Closed-Form Bounded



Velocity Solution - PMC - NIH, accessed February 1, 2026,
[htps://pmc.ncbi.nlm.nih.gov/articles/PMC8624698/](https://pmc.ncbi.nlm.nih.gov/articles/PMC8624698/)
5.​ Continuous trajectory planning for welding of complex joints using bezie curve. 


TUTDoR, accessed February 1, 2026,
[htps://tutvital.tut.ac.za/bitstreams/0987ba96-8f46-4f90-97a8-bf53c7d6c4e3/do](https://tutvital.tut.ac.za/bitstreams/0987ba96-8f46-4f90-97a8-bf53c7d6c4e3/download)
[wnload](https://tutvital.tut.ac.za/bitstreams/0987ba96-8f46-4f90-97a8-bf53c7d6c4e3/download)
6.​ Guide to Weaving in Welding: Weave Patterns and Pitch Calculation - Hirebotics



Blog, accessed February 1, 2026,
[htps://blog.hirebotics.com/weave-welding-guide](https://blog.hirebotics.com/weave-welding-guide)
7.​ Welding Weaving Patterns | HALDEN, accessed February 1, 2026,



[htps://haldencn.com/welding-weaving-paterns/](https://haldencn.com/welding-weaving-patterns/)
8.​ Weaving Condition Files - Yaskawa Knowledge Center, accessed February 1,



2026,
[htps://knowledge.motoman.com/hc/en-us/articles/5842915195543-Weaving-Con](https://knowledge.motoman.com/hc/en-us/articles/5842915195543-Weaving-Condition-Files)
[dition-Files](https://knowledge.motoman.com/hc/en-us/articles/5842915195543-Weaving-Condition-Files)
9.​ A novel 8-shape trajectory weaving welding control algorithm with auto-adjust

welding torch attitude | Request PDF - ResearchGate, accessed February 1, 2026,
[htps://www.researchgate.net/publication/360232137_A_novel_8-shape_trajector](https://www.researchgate.net/publication/360232137_A_novel_8-shape_trajectory_weaving_welding_control_algorithm_with_auto-adjust_welding_torch_attitude)
[y_weaving_welding_control_algorithm_with_auto-adjust_welding_torch_atitude](https://www.researchgate.net/publication/360232137_A_novel_8-shape_trajectory_weaving_welding_control_algorithm_with_auto-adjust_welding_torch_attitude)
10.​ Schematic diagram of weaving points of circular trajectory. - ResearchGate,



accessed February 1, 2026,
[htps://www.researchgate.net/fgure/Schematic-diagram-of-weaving-points-of-c](https://www.researchgate.net/figure/Schematic-diagram-of-weaving-points-of-circular-trajectory_fig3_353844809)


[ircular-trajectory_fg3_353844809](https://www.researchgate.net/figure/Schematic-diagram-of-weaving-points-of-circular-trajectory_fig3_353844809)
11.​ Eight Curve -- from Wolfram MathWorld, accessed February 1, 2026,



[htps://mathworld.wolfram.com/EightCurve.html](https://mathworld.wolfram.com/EightCurve.html)
12.​ Eight Curve (Leminiscate of Gerono) - Statistics How To, accessed February 1,

2026, [htps://www.statisticshowto.com/eight-curve/](https://www.statisticshowto.com/eight-curve/)
13.​ Weaving FANUC | PDF | Welding | Construction - Scribd, accessed February 1,



2026, [htps://www.scribd.com/document/942466275/Weaving-FANUC](https://www.scribd.com/document/942466275/Weaving-FANUC)
14.​ Adjusting Crater Fill with ESAB - Beacon Platform - Hirebotics, accessed February



1, 2026,
[htps://help.hirebotics.com/en/articles/8280465-adjusting-crater-fll-with-esab](https://help.hirebotics.com/en/articles/8280465-adjusting-crater-fill-with-esab)
15.​ MULTI NEXT TACK/STITCH - Koike, accessed February 1, 2026,



[htps://www.koike.com/content/Manuals/Wel-Handy%20Multi%20Next%20Manu](https://www.koike.com/content/Manuals/Wel-Handy%20Multi%20Next%20Manual%204_19.pdf)
[al%204_19.pdf](https://www.koike.com/content/Manuals/Wel-Handy%20Multi%20Next%20Manual%204_19.pdf)
16.​ TECH TIP: Programming The Crater Fill Function For Welding Aluminium On SP

Series MIG Welders, accessed February 1, 2026,
[htp://prospotwelding.blogspot.com/2018/06/tech-tip-programming-crater-fll.ht](http://prospotwelding.blogspot.com/2018/06/tech-tip-programming-crater-fill.html)
[ml](http://prospotwelding.blogspot.com/2018/06/tech-tip-programming-crater-fill.html)
17.​ Arc Shots: Close-Up Aluminum MIG Crater Fill Technique - YouTube, accessed



February 1, 2026, [htps://www.youtube.com/shorts/qlGp9iKzKOs](https://www.youtube.com/shorts/qlGp9iKzKOs)
18.​ Filling craters with MIG : r/Welding - Reddit, accessed February 1, 2026,



[htps://www.reddit.com/r/Welding/comments/1i5dow5/flling_craters_with_mig/](https://www.reddit.com/r/Welding/comments/1i5dow5/filling_craters_with_mig/)
19.​ Optimization of Welding Parameters Using an Improved Hill-Climbing Algorithm



Based on BP Neural Network for Multi-Bead Weld Smoothness Control - NIH,
accessed February 1, 2026, [htps://pmc.ncbi.nlm.nih.gov/articles/PMC12430673/](https://pmc.ncbi.nlm.nih.gov/articles/PMC12430673/)
20.​ COBEM2021-1752 WELDING BEADS OVERLAPPING ALGORITHM DEDICATED TO



WAAM - LABSOLDA, accessed February 1, 2026,
[htps://labsolda.ufsc.br/publicacoes/artigos/2021_cobem_rocha.pdf](https://labsolda.ufsc.br/publicacoes/artigos/2021_cobem_rocha.pdf)
21.​ CN111496428B - Multilayer multi-pass welding bead planning method based on



straight welding seam contour recognition and welding workstation - Google
Patents, accessed February 1, 2026,
[htps://patents.google.com/patent/CN111496428B/en](https://patents.google.com/patent/CN111496428B/en)
22.​ Multipass Welding Guide: Techniques, Applications and Best Practices, accessed

February 1, 2026, [htps://www.arccaptain.com/blogs/article/multipass-welding](https://www.arccaptain.com/blogs/article/multipass-welding)
23.​ I've been welding for about two months now and this is one of my first multi pass

fillet welds. Thought? - Reddit, accessed February 1, 2026,
[htps://www.reddit.com/r/Welding/comments/prp0i8/ive_been_welding_for_about](https://www.reddit.com/r/Welding/comments/prp0i8/ive_been_welding_for_about_two_months_now_and/)
[_two_months_now_and/](https://www.reddit.com/r/Welding/comments/prp0i8/ive_been_welding_for_about_two_months_now_and/)
24.​ (PDF) A knowledge-based multipass welding distortion estimation method for a

multi-robot welding off-line programming and simulation software ResearchGate, accessed February 1, 2026,
[htps://www.researchgate.net/publication/347073103_A_knowledge-based_multi](https://www.researchgate.net/publication/347073103_A_knowledge-based_multipass_welding_distortion_estimation_method_for_a_multi-robot_welding_off-line_programming_and_simulation_software)
[pass_welding_distortion_estimation_method_for_a_multi-robot_welding_of-line](https://www.researchgate.net/publication/347073103_A_knowledge-based_multipass_welding_distortion_estimation_method_for_a_multi-robot_welding_off-line_programming_and_simulation_software)
[_programming_and_simulation_sofware](https://www.researchgate.net/publication/347073103_A_knowledge-based_multipass_welding_distortion_estimation_method_for_a_multi-robot_welding_off-line_programming_and_simulation_software)
25.​ 04 - Welding Design., accessed February 1, 2026,



[htps://www.nrc.gov/docs/ML1215/ML12157A631.pdf](https://www.nrc.gov/docs/ML1215/ML12157A631.pdf)


