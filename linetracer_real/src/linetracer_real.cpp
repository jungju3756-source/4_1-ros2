#include "linetracer_real/linetracer_real.hpp"                                                  // LineTrackerProcessor 클래스 선언 및 의존 헤더 포함

// ============================================================================
// 생성자: 노드 초기화, 파라미터 선언, QoS 설정, 토픽 생성, 키보드 스레드 시작
// ============================================================================
LineTrackerProcessor::LineTrackerProcessor() : Node("line_tracker_node"), mode_(false), k_(0.13), base_vel_(150), running_(true) { // 부모 Node 이름 "line_tracker_node"로 초기화, 주행모드 OFF, 게인 0.13, 기본속도 150, 실행플래그 true
    this->declare_parameter("k", 0.14);                                                         // 런타임 파라미터 "k" 선언 (기본값 0.14: P 제어 게인)
    this->declare_parameter("base_vel", 120);                                                   // 런타임 파라미터 "base_vel" 선언 (기본값 120: 직진 기본 속도)

    k_ = this->get_parameter("k").as_double();                                                  // 파라미터 서버에서 "k" 값을 double로 읽어 멤버변수에 저장
    base_vel_ = this->get_parameter("base_vel").as_int();                                       // 파라미터 서버에서 "base_vel" 값을 int로 읽어 멤버변수에 저장
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();                             // 구독 QoS: 최근 10개 유지 + BestEffort(신뢰성보다 실시간성, 카메라 영상에 적합)
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));                                           // 퍼블리시 QoS: 최근 10개 유지 + Reliable(기본) — 모터 명령은 손실되면 안되므로 dxl_wsl과 동일

    raw_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(                    // CompressedImage 타입 구독자 생성
        "Image_Topic", sub_qos,                                                                 // 토픽 이름 "Image_Topic"에 위 QoS 적용
        std::bind(&LineTrackerProcessor::image_callback, this, std::placeholders::_1));         // 콜백 함수 바인딩: 메시지 수신 시 image_callback 호출

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", pub_qos);    // Vector3 타입 퍼블리셔 생성 (Pi5의 Dynamixel 드라이버가 구독)

    // 초기 추적 위치 설정
    last_line_x_ = 320.0;                                                                       // 직전 라인 중심 x좌표 초기값 (640px 가정한 화면 중앙)
    last_line_y_ = 45.0;                                                                        // 직전 라인 중심 y좌표 초기값 (ROI 영역 중앙 부근)

    RCLCPP_INFO(this->get_logger(), "분석 및 제어 노드 시작. 's': 주행, 'q': 정지");             // 노드 실행 안내 메시지 출력 (사용자에게 키 입력 방법 알림)

    key_thread_ = std::thread(&LineTrackerProcessor::keyboardLoop, this);                       // 키보드 입력 감지 스레드 시작 (image_callback과 병렬 실행)
}

// ============================================================================
// 소멸자: 키보드 스레드 안전 종료
// ============================================================================
LineTrackerProcessor::~LineTrackerProcessor() {                                                 // 노드 소멸 시 호출
    running_ = false;                                                                           // atomic 플래그를 false로 → keyboardLoop의 while문 탈출 유도
    if (key_thread_.joinable()) key_thread_.join();                                             // 스레드가 유효하면 종료 대기 (좀비 스레드 방지)
}

// ============================================================================
// 1. 전처리 함수: ROI 설정 + 그레이스케일 + 밝기 정규화 + 이진화
// ============================================================================
cv::Mat LineTrackerProcessor::setROI(cv::Mat &frame) {                                          // 원본 컬러 프레임을 참조로 받아 이진 ROI Mat 반환
    // 하단 1/4 영역 추출
    cv::Rect roi_rect(0, frame.rows * 3 / 4, frame.cols, frame.rows / 4);                       // ROI 사각형 정의: (x=0, y=전체높이×3/4, 폭=전체, 높이=전체/4) → 하단 1/4
    cv::Mat roi = frame(roi_rect).clone();                                                      // ROI 영역을 깊은 복사 (원본 frame 훼손 방지)


    cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);                                                 // BGR 3채널 → 그레이스케일 1채널 변환 (밝기만 사용)
    // ROI 내부 평균 밝기를 이용한 보정
    roi += cv::Scalar(100) - cv::mean(roi);                                                     // 밝기 정규화: 현재 평균값을 100으로 맞춤 → 조명 변화에 강건
    cv::threshold(roi, roi, 150, 255, cv::THRESH_BINARY);                                       // 이진화: 150 초과 픽셀을 255(흰색)로, 이하는 0(검정) → 밝은 라인 추출

    return roi;                                                                                 // 이진화된 ROI 반환
}

// ============================================================================
// 2. 라인 탐색 함수: Connected Component 분석 + 최근접 blob 추적
// ============================================================================
int LineTrackerProcessor::findLine(cv::Mat &bin_roi, cv::Mat &stats, cv::Mat &centroids) {      // 이진ROI, 출력용 stats/centroids 참조를 받아 유효 라인 인덱스 반환
    int n_labels = cv::connectedComponentsWithStats(bin_roi, labels_, stats, centroids);        // 연결요소 분석 → 레이블 수 반환 (labels_: 픽셀별 라벨, stats: 각 객체 정보, centroids: 중심점)

    // [1단계] 현재 중심점과 가장 가까운 후보 찾기
    int min_index = -1;                                                                         // 선택된 후보 인덱스 (-1 = 미선택)
    double min_dist = static_cast<double>(bin_roi.cols);                                        // 최소거리 초기값을 ROI 폭으로 설정 (큰 값으로 시작)

    for (int i = 1; i < n_labels; i++) {                                                        // 0번은 배경이므로 1번부터 순회
        int area = stats.at<int>(i, cv::CC_STAT_AREA);                                          // i번째 객체의 픽셀 면적 추출
        if (area > 100) { // 면적 50 이상만 취급                                                  // 면적 100 초과만 후보로 인정 (노이즈 제거)
            double cx = centroids.at<double>(i, 0);                                             // i번째 객체 중심 x좌표
            double cy = centroids.at<double>(i, 1);                                             // i번째 객체 중심 y좌표
            double dist = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_line_x_, last_line_y_)); // 직전 추적점과의 유클리드 거리 계산

            if (dist < min_dist && dist <= 150.0) {                                             // 현재 최소거리보다 작고, 절대거리 150px 이내인 경우
                min_dist = dist;                                                                // 최소거리 갱신
                min_index = i;                                                                  // 선택 인덱스 갱신
            }
        }
    }

    // 150px 이내에 후보가 있으면 중심점 1차 갱신
    if (min_index != -1 && min_dist <= 150.0) {                                                 // 유효 후보가 있으면
        last_line_x_ = centroids.at<double>(min_index, 0);                                      // 추적점 x좌표를 선택 객체 중심으로 갱신
        last_line_y_ = centroids.at<double>(min_index, 1);                                      // 추적점 y좌표를 선택 객체 중심으로 갱신
    }

    // [2단계] 갱신된 중심점에서 다시 한번 가장 가까운 blob 확정
    int idx = -1;                                                                               // 최종 선택 인덱스 초기화
    double best = static_cast<double>(bin_roi.cols);                                            // 최소거리 초기값 (큰 값)

    for (int i = 1; i < stats.rows; i++) {                                                      // 모든 객체 재순회 (배경 제외)
        double cx = centroids.at<double>(i, 0);                                                 // i번째 중심 x
        double cy = centroids.at<double>(i, 1);                                                 // i번째 중심 y
        double d = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_line_x_, last_line_y_));     // 갱신된 추적점과의 거리

        if (d < best) {                                                                         // 현재 최소거리보다 작으면
            best = d;                                                                           // 최소거리 갱신
            idx = i;                                                                            // 최종 인덱스 갱신
        }
    }

    // 최종 거리가 30px 이내일 때만 유효한 인덱스로 인정
    if (best > 30.0) {                                                                          // 최종 거리가 30px 초과 → 신뢰할 수 없음
        idx = -1;                                                                               // 무효 처리 (라인 손실 상태)
    }
    return idx;                                                                                 // 최종 라인 인덱스 반환 (-1 = 없음)
}

// ============================================================================
// 3. 시각화 함수: 검출된 라인 + 후보 노이즈 + 추적점 시각화
// ============================================================================
cv::Mat LineTrackerProcessor::drawResult(cv::Mat &bin_roi, cv::Mat &stats, int best_idx) {      // 이진ROI와 stats, 최종 인덱스를 받아 시각화 Mat 반환
    cv::Mat display;                                                                            // 결과 출력용 Mat 선언
    cv::cvtColor(bin_roi, display, cv::COLOR_GRAY2BGR);                                         // 흑백 ROI를 BGR 3채널로 변환 (컬러 도형을 그리기 위함)

    for (int i = 1; i < stats.rows; i++) {                                                      // 모든 객체 순회 (배경 제외)
        int area = stats.at<int>(i, cv::CC_STAT_AREA);                                          // 객체 면적
        if (area < 100) continue;                                                               // 면적 100 미만은 건너뜀 (시각적 노이즈 제거)

        int l = stats.at<int>(i, 0), t = stats.at<int>(i, 1), w = stats.at<int>(i, 2), h = stats.at<int>(i, 3); // 바운딩 박스 좌표 (left, top, width, height)

        if (i == best_idx) {                                                                    // 최종 선택된 라인이면
            // 검출된 라인 (빨간색)
            cv::rectangle(display, cv::Rect(l, t, w, h), cv::Scalar(0, 0, 255), 2);             // 빨간색 굵은 사각형 그리기 (BGR: 0,0,255)
        } else {                                                                                // 선택되지 않은 후보면
            // 후보 노이즈 (파란색)
            cv::rectangle(display, cv::Rect(l, t, w, h), cv::Scalar(255, 0, 0), 1);             // 파란색 얇은 사각형 그리기 (BGR: 255,0,0)
        }
    }
    // 최종 추적 지점 점 찍기
    cv::circle(display, cv::Point(static_cast<int>(last_line_x_), static_cast<int>(last_line_y_)), 3, cv::Scalar(0, 0, 255), -1); // 추적점에 반지름 3의 빨간 원(-1=채움) 표시

    return display;                                                                             // 시각화 결과 반환
}

// ============================================================================
// 4. 메인 콜백: 이미지 수신 → 전처리 → 라인 검출 → P 제어 → 퍼블리시
// ============================================================================
void LineTrackerProcessor::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) { // 압축 이미지 메시지를 shared_ptr로 수신
    auto startTime = std::chrono::steady_clock::now();                                          // 처리 시작 시각 기록 (성능 측정용)

    k_ = this->get_parameter("k").as_double();                                                  // 매 프레임마다 파라미터 재읽기 (런타임 튜닝 반영)
    base_vel_ = this->get_parameter("base_vel").as_int();                                       // 매 프레임마다 base_vel 재읽기



    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);                         // JPEG 바이트 배열을 BGR 이미지로 디코딩
    if (frame.empty()) return;                                                                  // 디코딩 실패 시 조기 반환 (빈 프레임 방지)

    // 전처리
    cv::Mat bin_roi = setROI(frame);                                                            // ROI 추출 + 이진화
    cv::Mat stats, centroids;                                                                   // 연결요소 분석 결과 출력용 Mat
    // 라인 검출
    int best_idx = findLine(bin_roi, stats, centroids);                                         // 라인 탐색 → 유효 인덱스 반환
    //
    cv::Mat display = drawResult(bin_roi, stats, best_idx);                                     // 디버그용 시각화 이미지 생성


    // 오차 계산 및 발행
    double error = (bin_roi.cols / 2.0) - last_line_x_;                                         // 오차 = 이미지 중앙 x − 현재 라인 x (양수=라인이 왼쪽에 있음 → 좌회전 필요)
    geometry_msgs::msg::Vector3 vel_msg;                                                        // 속도 명령 메시지 인스턴스 생성
    if (mode_) {                                                                                // 주행 모드 ON이면
        vel_msg.x = base_vel_ - error * k_;                                                     // 좌측 모터: 기본속도 − 오차×게인 (오차가 크면 좌측 감속)
        vel_msg.y = -(base_vel_ + error * k_);                                                  // 우측 모터: 부호 반전 (차동 구동: 좌우 반대 회전으로 직진/회전)
    } else {                                                                                    // 정지 모드면
        vel_msg.x = 0; vel_msg.y = 0;                                                           // 좌우 모터 모두 0 (정지)
    }
    vel_pub_->publish(vel_msg);                                                                 // topic_dxlpub 토픽으로 속도 명령 퍼블리시

    auto endTime = std::chrono::steady_clock::now();                                            // 처리 종료 시각 기록
    float totalTime = std::chrono::duration<float, std::milli>(endTime - startTime).count();    // 총 처리 시간(ms) 계산
    RCLCPP_INFO(this->get_logger(), "err:%.2lf lvel:%.2f rvel:%.2f time:%.2f", error,vel_msg.x,vel_msg.y, totalTime); // 오차/좌속/우속/처리시간 로그 출력


    cv::imshow("1. Raw Video", frame);                                                          // 원본 영상 창 표시
    cv::imshow("2. Binary Debug", display);                                                     // 이진화+시각화 창 표시
    cv::waitKey(1);                                                                             // 1ms 대기 (imshow가 화면 갱신되도록 하는 필수 호출)
}


// ============================================================================
// 5. 키보드 입력 스레드: 터미널 raw 모드로 's'/'q' 감지
// ============================================================================
// 별도 스레드에서 키보드 입력 대기
void LineTrackerProcessor::keyboardLoop() {                                                     // 별도 스레드 엔트리 함수
    struct termios oldt, newt;                                                                  // 터미널 설정 저장용 구조체 (원본/수정본)
    tcgetattr(STDIN_FILENO, &oldt);                                                             // 현재 터미널 속성을 oldt에 백업
    newt = oldt;                                                                                // 수정용 구조체에 복사
    newt.c_lflag &= ~(ICANON | ECHO);                                                           // canonical(줄단위) 모드와 echo 해제 → 문자 단위 즉시 입력, 화면 출력 안함
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);                                                    // 변경된 속성을 즉시 적용

    while (running_) {                                                                          // 노드 실행 중인 동안 반복 (소멸자에서 false로 설정)
        fd_set fds;                                                                             // select용 파일 디스크립터 집합
        FD_ZERO(&fds);                                                                          // 집합 초기화
        FD_SET(STDIN_FILENO, &fds);                                                             // 표준입력(fd 0)을 감시 대상에 추가
        struct timeval tv = {0, 100000}; // 100ms timeout                                       // select 타임아웃: 0초 100000us = 100ms
        if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {                        // 100ms 내에 입력이 들어왔으면
            char ch = getchar();                                                                // 한 문자 읽기 (canonical 해제로 버퍼링 없음)
            if (ch == 'q') { mode_ = false; RCLCPP_WARN(this->get_logger(), "STOP"); }          // 'q' 입력 → 주행 정지 + 경고 로그
            else if (ch == 's') { mode_ = true; RCLCPP_INFO(this->get_logger(), "START"); }     // 's' 입력 → 주행 시작 + 정보 로그
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);                                                    // 스레드 종료 시 터미널 속성을 원본으로 복구 (셸 깨짐 방지)
}
