# **Báo cáo Kỹ thuật Chuyên sâu: Triển khai** **Giải thuật Động học Nghịch đảo Giải tích** **(Analytical IK) trên Nền tảng Robotics** **Library (RL) cho Robot Hàn 6-DOF**
## **1. Tổng quan Điều hành**

Báo cáo này được biên soạn nhằm phục vụ nhu cầu phát triển phần mềm điều khiển cốt lõi
(Core Math Engine) cho hệ thống robot hàn thương mại 6 bậc tự do (6-DOF). Dựa trên yêu
cầu kỹ thuật về việc sử dụng Robotics Library (RL) làm nền tảng toán học và cấu trúc hình học
tương đồng với PUMA 560, tài liệu này cung cấp một lộ trình triển khai toàn diện từ thiết kế cơ
khí (CAD) đến mã nguồn điều khiển thời gian thực (C++).


Trong ứng dụng hàn hồ quang công nghiệp, độ chính xác của quỹ đạo (path accuracy) và tính
ổn định của vận tốc đầu mỏ hàn (TCP velocity) là yếu tố sống còn. Các giải thuật động học
nghịch đảo (IK) dựa trên phương pháp số (Numerical Methods) như Newton-Raphson hay
Levenberg-Marquardt, mặc dù linh hoạt, thường gặp vấn đề về thời gian hội tụ không xác định
và rủi ro rơi vào cực tiểu địa phương (local minima). [1] Ngược lại, giải thuật động học nghịch đảo
giải tích (Analytical IK), cụ thể là lớp rl::kin::Puma trong Robotics Library, cung cấp lời giải
chính xác, thời gian tính toán tất định và khả năng kiểm soát tường minh các cấu hình đa
nghiệm (Arm, Elbow, Wrist). [3 ]


Tài liệu này sẽ phân tích sâu các khía cạnh:


1.​ **Chuyển đổi dữ liệu** : Quy trình trích xuất tham số Denavit-Hartenberg (DH) chuẩn từ bản

vẽ CAD và ánh xạ vào định dạng XML của RL.
2.​ **Kiến trúc phần mềm** : Phương pháp "ép" hệ thống sử dụng rl::kin::Puma thay vì các bộ

giải số học mặc định.
3.​ **Xử lý đa nghiệm** : Logic toán học để lọc 8 nghiệm khả dĩ dựa trên các cờ cấu hình và

tránh va chạm.
4.​ **Hiện thực hóa** : Cung cấp mã nguồn C++ mẫu đạt chuẩn công nghiệp.

## **2. Cơ sở Lý thuyết: Cấu trúc PUMA 560 và Ràng buộc** **Hình học**


Để sử dụng thành công lớp rl::kin::Puma, robot tùy chỉnh bắt buộc phải tuân thủ nghiêm ngặt
các ràng buộc hình học mà giải thuật này giả định. Việc hiểu rõ bản chất toán học của PUMA


560 là bước đầu tiên để đảm bảo tính khả thi của dự án.

### **2.1. Đặc trưng Tô-pô và Tiêu chuẩn Pieper**


Robot PUMA (Programmable Universal Machine for Assembly) là đại diện kinh điển của robot
chuỗi nối tiếp có cổ tay cầu (spherical wrist). Theo tiêu chuẩn Pieper, một robot 6-DOF sẽ có
lời giải IK dạng khép kín (closed-form) nếu ba trục quay liên tiếp cắt nhau tại một điểm. [1 ]


Trong cấu trúc PUMA 560 và robot hàn của dự án:


●​ **Khớp 1, 2, 3 (Cơ cấu định vị)** : Chịu trách nhiệm đưa tâm cổ tay (Wrist Center - WC) đến

vị trí mong muốn.
●​ **Khớp 4, 5, 6 (Cơ cấu định hướng)** : Các trục quay của ba khớp này phải giao nhau tại

một điểm duy nhất. Điều này cho phép tách rời bài toán vị trí và bài toán hướng. [6 ]


**Ràng buộc hình học bắt buộc đối với rl::kin::Puma:** Dựa trên mã nguồn và tài liệu của RL [3],
robot tùy chỉnh phải thỏa mãn:


1.​ **Trục 1 vuông góc với Trục 2** : ( ).


2.​ **Trục 2 song song với Trục 3** : ( ). Đây là đặc điểm quan trọng giúp đơn giản
hóa phương trình lượng giác của vai và khuỷu.


3.​ **Trục 3 vuông góc với Trục 4** : ( ).
4.​ **Giao điểm cổ tay** : Trục 4, 5, và 6 phải đồng quy. Nếu thiết kế cơ khí của robot hàn có độ

lệch (offset) tại khớp 4 hoặc 5 (thường thấy ở một số robot sơn hoặc hàn rỗng ruột hiện
đại), giải thuật rl::kin::Puma sẽ **không chính xác** .
5.​ **Độ lệch vai (Shoulder Offset)** : PUMA 560 có độ lệch giữa tâm quay khớp 1 và mặt


phẳng quay của khớp 2/3 (tham số trong DH). rl::kin::Puma được thiết kế đặc biệt để
xử lý tham số này, khác với các robot phẳng hoàn toàn.

### **2.2. Hệ tham số Denavit-Hartenberg (DH)**


Robotics Library sử dụng quy ước DH Chuẩn (Standard DH), không phải DH Sửa đổi
(Modified/Craig's DH). [3] Sự nhầm lẫn giữa hai quy ước này là nguyên nhân hàng đầu dẫn đến
sai lệch mô hình.


Trong quy ước DH Chuẩn được RL áp dụng, phép biến đổi từ khung sang khung được
mô tả bởi ma trận:


Bảng dưới đây tóm tắt ý nghĩa vật lý của các tham số trong ngữ cảnh robot hàn PUMA-like:


|Tham số|Ý nghĩa Vật lý|Lưu ý khi trích xuất từ<br>CAD|
|---|---|---|
||Góc quay khớp|Biến số điều khiển (Joint<br>Angle). Cần xác định vị trí<br>Zero (Home position) chính<br>xác.|
||Độ lệch dọc trục<br>|Khoảng cách giữa hai<br>đường vuông góc chung<br>dọc theo trục khớp. PUMA<br>thường có<br>.|
||Độ dài liên kết (Link length)|Khoảng cách ngắn nhất<br>giữa trục<br> và<br> dọc<br>theo trục<br>.|
||Góc xoắn liên kết (Link<br>twist)|Góc giữa trục<br> và<br> <br>đo quanh trục<br>. Với<br>PUMA, giá trị thường là<br>.|

## **3. Quy trình Kỹ thuật: Từ CAD đến Mapping XML**

Đây là quy trình chuyển đổi dữ liệu hình học tĩnh từ phần mềm thiết kế (SolidWorks, Inventor)
sang định dạng XML động mà Robotics Library có thể hiểu và xử lý.

### **3.1. Bước 1: Trích xuất Dữ liệu từ CAD**


Kỹ sư cần thực hiện gán hệ tọa độ trên mô hình 3D theo quy tắc "Tay phải" (Right-Hand Rule)
và tuân thủ quy ước DH chuẩn:


1.​ **Gốc tọa độ (Base Frame - 0)** : Đặt tại chân đế robot. Trục trùng với trục quay khớp
1.
2.​ **Khung 1** : Gốc tại giao điểm trục 1 và trục 2. Nếu không cắt nhau (do shoulder offset), gốc


nằm trên trục 1 tại điểm vuông góc chung. vuông góc với và .


3.​ **Khung 2** : Gốc tại tâm khớp 2. trùng trục quay khớp 2.


4.​ **Khung 3** : Gốc tại tâm khớp 3. trùng trục quay khớp 3.
5.​ **Khung 4, 5, 6** : Gốc chung tại tâm giao điểm cổ tay (Wrist Center).


**Danh sách tham số cần đo kiểm:**


●​ **Chiều cao bệ (** **)** : Từ sàn đến tâm trục 2.


●​ **Độ lệch vai (** **hoặc** **tùy cách đặt)** : Khoảng cách ngang giữa trục 1 và mặt phẳng
cánh tay đòn.


●​ **Độ dài bắp tay (** **)** : Khoảng cách tâm trục 2 đến tâm trục 3.


●​ **Độ lệch khuỷu (** **)** : Khoảng cách từ mặt phẳng khuỷu đến tâm cổ tay.


●​ **Độ dài cẳng tay (** **)** : Thường xấp xỉ 0 hoặc rất nhỏ trên PUMA.

### **3.2. Bước 2: Định nghĩa File XML cho RL**


Robotics Library sử dụng định dạng XML tùy biến để mô tả mô hình động lực học và động học
(.rlmdl.xml). [7] Cấu trúc file này phản ánh cấu trúc cây (tree structure) của các liên kết.


Để đảm bảo rl::kin::Puma hoạt động, file XML phải chứa đầy đủ các thẻ mô tả tham số DH.
Dưới đây là mẫu file XML chuẩn cho robot hàn tùy chỉnh cấu trúc PUMA:


XML


​

​

​

​

​

​


**Lưu ý quan trọng về đơn vị:** Mặc dù XML có thể ghi độ, nhưng trong code C++, RL sử dụng
**Radian** và **Mét** . [8] Bộ nạp (Loader) của RL thường tự động chuyển đổi nếu đơn vị được khai
báo, nhưng tốt nhất nên chuẩn hóa đầu vào.

## **4. Kích hoạt Analytical IK (rl::kin::Puma)**


Một thách thức lớn khi sử dụng RL là rl::mdl::XmlFactory mặc định sẽ tạo ra một đối tượng
rl::mdl::Kinematic chung, sử dụng giải thuật số (Jacobian Inverse Kinematics) cho hàm
calculateInverse(). Để sử dụng giải thuật giải tích của PUMA, chúng ta cần "ép" hệ thống khởi
tạo lớp dẫn xuất rl::kin::Puma.

### **4.1. Vấn đề của Factory Mặc định**


Khi gọi rl::mdl::XmlFactory::create(), thư viện sẽ phân tích file XML. Nếu không có chỉ thị đặc
biệt, nó xây dựng chuỗi động học (kinematic chain) từ các khớp quay/trượt cơ bản. Lớp này
không biết rằng robot có cấu trúc PUMA đặc biệt để áp dụng công thức giải tích. [9 ]

### **4.2. Giải pháp: Ép kiểu và Khởi tạo Trực tiếp**


Có hai phương pháp để tích hợp rl::kin::Puma vào Core Engine của bạn:


**Phương pháp A: Kế thừa và Nạp chồng (Khuyên dùng cho tính linh hoạt)**


Tạo một lớp quản lý robot hàn riêng, lớp này sở hữu một đối tượng rl::kin::Puma. Thay vì dùng
Factory để tạo toàn bộ Model, bạn dùng Factory để đọc dữ liệu, sau đó copy tham số vào đối
tượng Puma thủ công hoặc dùng cơ chế nạp chồng.


**Phương pháp B: Sử dụng rl::kin::Puma trực tiếp trong C++**


Vì kích thước robot là tùy chỉnh nhưng cấu trúc là cố định, cách tối ưu nhất về hiệu năng là
khởi tạo trực tiếp lớp rl::kin::Puma và nạp tham số từ file cấu hình (hoặc hardcode nếu robot là
duy nhất).


Điều kiện tiên quyết để rl::kin::Puma::calculateInverse hoạt động chính xác:


●​ Gọi hàm setParameters() (nếu có) hoặc thiết lập trực tiếp các biến thành viên đại diện


cho độ dài liên kết ( ).
●​ Đảm bảo file header #include <rl/kin/Puma.h> được bao gồm. [4 ]

## **5. Xử lý Đa nghiệm và Cờ Cấu hình (Configuration** **Flags)**


Với robot 6-DOF PUMA, một vị trí và hướng của mỏ hàn (End-Effector Pose) có thể đạt được
bởi tối đa **8 bộ nghiệm** khác nhau. Việc chọn nghiệm nào là bài toán tối quan trọng trong hàn
để tránh va chạm và duy trì sự liên tục của đường hàn.

### **5.1. Định nghĩa 8 Bộ nghiệm (The Geometric Multiverse)**


Các nghiệm được phân loại dựa trên 3 chỉ số nhị phân (flags):



|Cờ Cấu hình (Flag)|Giá trị (Bit)|Mô tả Hình học|Biểu diễn Toán<br>học trong RL|
|---|---|---|---|
|**ARM**|RIGHT / LEFT|Xác định xem vai<br>robot đang ở bên<br>phải hay trái của<br>đường thẳng nối<br>gốc và cổ tay.|Dấu của<br> và hình<br>chiếu của<br> lên<br>mặt phẳng<br>.|
|**ELBOW**|UP / DOWN|Xác định khuỷu tay<br>chổng lên hay chúc<br>xuống so với đường<br>nối vai-cổ tay.|Dấu của<br>.|
|**WRIST**|FLIP / NO-FLIP|Xác định cổ tay có<br>bị "lật" ngược để<br>đạt cùng một<br>hướng hay không.|Dấu của<br> <br>(thường liên quan<br>đến việc<br> dương<br>hay âm).|


Tổng hợp: nghiệm.




### **5.2. Cấu trúc Trả về của calculateInverse**

Hàm rl::kin::Puma::calculateInverse(const Transform& x, VectorList& q) không trả về một
vector đơn lẻ.


●​ **Input** : x (Ma trận biến đổi đồng nhất 4x4 của mỏ hàn).
●​ **Output** : q (Danh sách std::vector chứa các rl::math::Vector).
●​ **Return** : bool (True nếu tìm thấy ít nhất 1 nghiệm).

### **5.3. Chiến lược Lọc nghiệm (Filtering Strategy)**


Kỹ sư trưởng cần xây dựng một bộ lọc (Wrapper) bọc lấy hàm calculateInverse. Bộ lọc này
thực hiện:


1.​ **Lọc giới hạn khớp (Joint Limit Check)** : Loại bỏ các nghiệm mà nằm ngoài [min,
max] quy định trong XML.
2.​ **Lọc theo Cờ (Flag Matching)** : Chỉ giữ lại các nghiệm khớp với cấu hình mong muốn (ví

dụ: ARM_LEFT, ELBOW_UP để hàn trần).
3.​ **Tối ưu hóa hành trình (Continuity Check)** : Trong số các nghiệm hợp lệ còn lại, chọn


nghiệm gần nhất với vị trí khớp hiện tại ( nhỏ nhất) để đảm bảo robot không thực
hiện các chuyển động giật cục nguy hiểm.

## **6. Triển khai Mã nguồn C++ (Reference** **Implementation)**


Dưới đây là mã nguồn C++ mẫu mô phỏng lớp WeldingRobotKinematics. Mã này minh họa
cách khởi tạo rl::kin::Puma, nạp tham số (giả định từ XML), và thực hiện giải IK có lọc cờ.

### **6.1. File Header: WeldingRobotKinematics.h**


C++


​

​


​

​

​

​

​

​


rl::math::Vector maxLimits;​
​

​
# **endif** // WELDING_ROBOT_KINEMATICS_H​

### **6.2. File Implementation: WeldingRobotKinematics.cpp**


C++


​

​

​

​
WeldingRobotKinematics::~WeldingRobotKinematics() {}​
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
if (!found |​
​

​

​

​

​

​

​

​
if (distance < minDistance) {​


​

## **7. Tích hợp Tránh va chạm (Collision Avoidance)**


Trong môi trường hàn, việc chỉ giải IK là chưa đủ. Mỏ hàn có thể va vào đồ gá hoặc phôi hàn.
Robotics Library cung cấp module rl::sg (Scene Graph) để giải quyết vấn đề này. [11 ]


Quy trình tích hợp vào vòng lặp điều khiển:


1.​ **Bước 1** : Nhận allSolutions từ solveAnalyticalIK.
2.​ **Bước 2** : Thay vì chọn ngay nghiệm tốt nhất, hãy tạo một danh sách "Ứng viên Tiềm năng"

(Candidate List).
3.​ **Bước 3** : Với mỗi ứng viên, cập nhật mô hình va chạm: ​


4.​ **Bước 4** : Kiểm tra va chạm với môi trường: ​


5.​ **Bước 5** : Chọn nghiệm tốt nhất từ danh sách ứng viên không va chạm còn lại.

## **8. Kết luận và Khuyến nghị**


Việc chuyển đổi từ một robot hàn tùy chỉnh sang mô hình toán học rl::kin::Puma là một quy
trình đòi hỏi sự chính xác tuyệt đối trong khâu ánh xạ DH. Bằng cách sử dụng trực tiếp lớp


rl::kin::Puma và bỏ qua cơ chế Factory mặc định, hệ thống điều khiển sẽ đạt được hiệu suất
tính toán thời gian thực và độ ổn định cần thiết cho các đường hàn phức tạp.


**Các bước hành động ngay:**


1.​ Kiểm tra lại bản vẽ CAD để xác nhận các điều kiện trực giao/song song của trục 1, 2, 3 và

sự đồng quy của trục 4, 5, 6.
2.​ Xây dựng file XML theo mẫu đã cung cấp, chú ý đơn vị đo.
3.​ Triển khai lớp C++ Wrapper (WeldingRobotKinematics) để quản lý logic lọc nghiệm.
4.​ Thực hiện Unit Test: So sánh kết quả của rl::kin::Puma::calculateForward (FK) và


calculateInverse (IK) để đảm bảo sai số vị trí .


Giải pháp này đảm bảo robot hàn của bạn không chỉ hoạt động chính xác mà còn có khả năng
xử lý thông minh các tình huống đa nghiệm, đặc trưng của các hệ thống robot công nghiệp
cao cấp.


**Works cited**



1.​ How to set up KDL (Kinematics and Dynamics Library) for Solving Kinematics of



Serial Chain Manipulators in C++ with ROS2 - FusyBots, accessed February 1,
2026,
[htps://www.fusybots.com/post/how-to-set-up-kdl-kinematics-and-dynamics-lib](https://www.fusybots.com/post/how-to-set-up-kdl-kinematics-and-dynamics-library-for-solving-kinematics-of-serial-chain-manipulat)
[rary-for-solving-kinematics-of-serial-chain-manipulat](https://www.fusybots.com/post/how-to-set-up-kdl-kinematics-and-dynamics-library-for-solving-kinematics-of-serial-chain-manipulat)
2.​ Inverse Kinematics in Robotics: What You Need to Know - RoboDK blog, accessed



February 1, 2026,
[htps://robodk.com/blog/inverse-kinematics-in-robotics-what-you-need-to-kno](https://robodk.com/blog/inverse-kinematics-in-robotics-what-you-need-to-know/)
[w/](https://robodk.com/blog/inverse-kinematics-in-robotics-what-you-need-to-know/)
3.​ rl/kin/Puma.cpp File Reference - Robotics Library, accessed February 1, 2026,



[htps://doc.roboticslibrary.org/0.7.0/d9/d15/_puma_8cpp.html](https://doc.roboticslibrary.org/0.7.0/d9/d15/_puma_8cpp.html)
4.​ rl/kin/Puma.h File Reference - Robotics Library, accessed February 1, 2026,



[htps://doc.roboticslibrary.org/0.6.2/d6/dc9/_puma_8h.html](https://doc.roboticslibrary.org/0.6.2/d6/dc9/_puma_8h.html)
5.​ Geometric Approach in Solving Inverse Kinematics of PUMA Robots - SciSpace,



accessed February 1, 2026,
[htps://scispace.com/pdf/geometric-approach-in-solving-inverse-kinematics-of-](https://scispace.com/pdf/geometric-approach-in-solving-inverse-kinematics-of-puma-ph4ngr22yh.pdf)
[puma-ph4ngr22yh.pdf](https://scispace.com/pdf/geometric-approach-in-solving-inverse-kinematics-of-puma-ph4ngr22yh.pdf)
6.​ Kinematic diagram of the PUMA robot. - ResearchGate, accessed February 1,

2026,
[htps://www.researchgate.net/fgure/Kinematic-diagram-of-the-PUMA-robot_fg](https://www.researchgate.net/figure/Kinematic-diagram-of-the-PUMA-robot_fig1_263284372)
[1_263284372](https://www.researchgate.net/figure/Kinematic-diagram-of-the-PUMA-robot_fig1_263284372)
7.​ Create a Robot Model - Robotics Library, accessed February 1, 2026,



[htps://www.roboticslibrary.org/tutorials/create-a-robot-model/](https://www.roboticslibrary.org/tutorials/create-a-robot-model/)
8.​ First Steps with RL on Windows - Robotics Library, accessed February 1, 2026,



[htps://www.roboticslibrary.org/tutorials/frst-steps-windows/](https://www.roboticslibrary.org/tutorials/first-steps-windows/)
9.​ API - Robotics Library, accessed February 1, 2026,



[htps://www.roboticslibrary.org/api](https://www.roboticslibrary.org/api)


10.​ First Steps with RL on Linux - Robotics Library, accessed February 1, 2026,



[htps://www.roboticslibrary.org/tutorials/frst-steps-linux/](https://www.roboticslibrary.org/tutorials/first-steps-linux/)
11.​ Robotics Library: Robotics Library, accessed February 1, 2026,



[htps://doc.roboticslibrary.org/](https://doc.roboticslibrary.org/)
12.​ Robotics Library - mediaTUM, accessed February 1, 2026,



[htps://mediatum.ub.tum.de/doc/1287151/581170.pdf](https://mediatum.ub.tum.de/doc/1287151/581170.pdf)


