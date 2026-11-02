# 그림과 영상 출처

본문의 외부 사진과 화면은 개념을 설명하기 위한 참고 자료입니다. Cleany의 실물 또는 시뮬레이션 결과는 각 자료의 캡션에서 별도로 표시합니다. 아래 파일은 2026-10-13에 확보했으며, 원본을 자르거나 색상·내용을 바꾸지 않았습니다. 웹 자료는 저장소의 revision을 고정해 보관했고, 파일별 원본 URL과 SHA-256은 저장소의 `assets/media/sources.json`에 기록했습니다.

## 사진과 움짤

| 본문 | 자료 | 작성자와 원본 | 이용 조건 |
| --- | --- | --- | --- |
| 06. SLAM | [2D 지도 생성 움짤](media/cartographer-map.gif) | [The Cartographer Authors](https://github.com/cartographer-project/cartographer_ros/blob/c138034db0c47fe0ea5a2abe516acae02190dbf5/docs/source/demo_2d.gif) | [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0), 원본 그대로 |
| 04. TF | [데모의 TF 출력](media/cartographer-frames.jpg) | [The Cartographer Authors](https://github.com/cartographer-project/cartographer_ros/blob/c138034db0c47fe0ea5a2abe516acae02190dbf5/docs/source/frames_demo_2d.jpg) | [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0), 원본 그대로 |
| 08. Grasp | [단계별 계획 시각화 움짤](media/moveit-stages.gif) | [Willow Garage / PickNik](https://github.com/moveit/moveit2_tutorials/blob/b8f954ce045b00f29de4da24146eb7732e7b2a9e/doc/examples/moveit_task_constructor/mtc_show_stages.gif) | [BSD-3-Clause](https://opensource.org/license/bsd-3-clause), 원본 그대로 |
| 07. RGB-D | [깊이 영상 화면](media/realsense-viewer.png) | [RealSense SDK contributors](https://github.com/realsenseai/librealsense/blob/c84fe6438bd14a4cc69f46deae92c993268232c6/doc/img/windows_viewer_preview.PNG) | [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0), 원본 그대로 |
| 07. RGB-D | [색상 점군 화면](media/realsense-pointcloud.png) | [RealSense SDK contributors](https://github.com/realsenseai/librealsense/blob/c84fe6438bd14a4cc69f46deae92c993268232c6/doc/img/windows_pointcloud_preview.PNG) | [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0), 원본 그대로 |
| 09. 보정 | [ChArUco 코너 검출 사진](media/charuco-corners.jpg) | [OpenCV contributors](https://github.com/opencv/opencv_contrib/blob/c7602a8f74205e44389bd6a4e8d727d32e7e27b4/modules/aruco/tutorials/charuco_detection/images/chcorners.jpg) | [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0), 원본 그대로 |
| 09. 보정 | [ChArUco 자세 축 사진](media/charuco-pose.jpg) | [OpenCV contributors](https://github.com/opencv/opencv_contrib/blob/c7602a8f74205e44389bd6a4e8d727d32e7e27b4/modules/aruco/tutorials/charuco_detection/images/chaxis.jpg) | [Apache-2.0](https://www.apache.org/licenses/LICENSE-2.0), 원본 그대로 |
| 05. 베이스 | [메카넘 롤러 사진](media/mecanum-wheels.jpg) | [KillerGardevoir](https://commons.wikimedia.org/wiki/File:MG_6646.resize.jpg) | [CC-BY-3.0](https://creativecommons.org/licenses/by/3.0/), 원본 그대로 |

RealSense의 두 화면은 서로 다른 장면입니다. 나란히 비교한 같은 촬영의 전후 결과가 아닙니다. MoveIt 자료는 Task Constructor 데모이며 Cleany의 selector 구현이나 물리 실행 결과가 아닙니다. Cartographer TF 화면의 발행 주체는 Cleany의 Gazebo 경로와 다릅니다.

## 영상 링크

01장의 [XLeRobot 조립 영상](https://www.youtube.com/watch?v=upB1CEFeOlk)은 [XLeRobot 공식 저장소](https://github.com/Vector-Wangel/XLeRobot)에서 소개하는 WOWROBO 자료입니다. 영상과 썸네일을 복제하지 않고 원본 링크로 제공합니다. 재생에는 인터넷 연결이 필요합니다.

## 라이선스 원문

이 페이지와 아래 라이선스 원문은 배포 HTML에도 포함됩니다. 외부 자산의 라이선스는 이 가이드의 글이나 직접 작성한 SVG의 라이선스를 바꾸지 않습니다.

Cartographer: Copyright 2016 The Cartographer Authors. OpenCV: OpenCV contributors. RealSense SDK: RealSense SDK contributors (formerly Intel Corporation). 각 자산의 출처는 위 원본 링크를 참조하세요. RealSense의 SDK 의존성 목록인 NOTICE.md에는 별도 프로그램들의 고지가 있으며, 여기서는 이미지 파일만 사용합니다.

<details><summary>Apache License 2.0 — Cartographer, RealSense, OpenCV 자료</summary>

```text

                                 Apache License
                           Version 2.0, January 2004
                        http://www.apache.org/licenses/

   TERMS AND CONDITIONS FOR USE, REPRODUCTION, AND DISTRIBUTION

   1. Definitions.

      "License" shall mean the terms and conditions for use, reproduction,
      and distribution as defined by Sections 1 through 9 of this document.

      "Licensor" shall mean the copyright owner or entity authorized by
      the copyright owner that is granting the License.

      "Legal Entity" shall mean the union of the acting entity and all
      other entities that control, are controlled by, or are under common
      control with that entity. For the purposes of this definition,
      "control" means (i) the power, direct or indirect, to cause the
      direction or management of such entity, whether by contract or
      otherwise, or (ii) ownership of fifty percent (50%) or more of the
      outstanding shares, or (iii) beneficial ownership of such entity.

      "You" (or "Your") shall mean an individual or Legal Entity
      exercising permissions granted by this License.

      "Source" form shall mean the preferred form for making modifications,
      including but not limited to software source code, documentation
      source, and configuration files.

      "Object" form shall mean any form resulting from mechanical
      transformation or translation of a Source form, including but
      not limited to compiled object code, generated documentation,
      and conversions to other media types.

      "Work" shall mean the work of authorship, whether in Source or
      Object form, made available under the License, as indicated by a
      copyright notice that is included in or attached to the work
      (an example is provided in the Appendix below).

      "Derivative Works" shall mean any work, whether in Source or Object
      form, that is based on (or derived from) the Work and for which the
      editorial revisions, annotations, elaborations, or other modifications
      represent, as a whole, an original work of authorship. For the purposes
      of this License, Derivative Works shall not include works that remain
      separable from, or merely link (or bind by name) to the interfaces of,
      the Work and Derivative Works thereof.

      "Contribution" shall mean any work of authorship, including
      the original version of the Work and any modifications or additions
      to that Work or Derivative Works thereof, that is intentionally
      submitted to Licensor for inclusion in the Work by the copyright owner
      or by an individual or Legal Entity authorized to submit on behalf of
      the copyright owner. For the purposes of this definition, "submitted"
      means any form of electronic, verbal, or written communication sent
      to the Licensor or its representatives, including but not limited to
      communication on electronic mailing lists, source code control systems,
      and issue tracking systems that are managed by, or on behalf of, the
      Licensor for the purpose of discussing and improving the Work, but
      excluding communication that is conspicuously marked or otherwise
      designated in writing by the copyright owner as "Not a Contribution."

      "Contributor" shall mean Licensor and any individual or Legal Entity
      on behalf of whom a Contribution has been received by Licensor and
      subsequently incorporated within the Work.

   2. Grant of Copyright License. Subject to the terms and conditions of
      this License, each Contributor hereby grants to You a perpetual,
      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
      copyright license to reproduce, prepare Derivative Works of,
      publicly display, publicly perform, sublicense, and distribute the
      Work and such Derivative Works in Source or Object form.

   3. Grant of Patent License. Subject to the terms and conditions of
      this License, each Contributor hereby grants to You a perpetual,
      worldwide, non-exclusive, no-charge, royalty-free, irrevocable
      (except as stated in this section) patent license to make, have made,
      use, offer to sell, sell, import, and otherwise transfer the Work,
      where such license applies only to those patent claims licensable
      by such Contributor that are necessarily infringed by their
      Contribution(s) alone or by combination of their Contribution(s)
      with the Work to which such Contribution(s) was submitted. If You
      institute patent litigation against any entity (including a
      cross-claim or counterclaim in a lawsuit) alleging that the Work
      or a Contribution incorporated within the Work constitutes direct
      or contributory patent infringement, then any patent licenses
      granted to You under this License for that Work shall terminate
      as of the date such litigation is filed.

   4. Redistribution. You may reproduce and distribute copies of the
      Work or Derivative Works thereof in any medium, with or without
      modifications, and in Source or Object form, provided that You
      meet the following conditions:

      (a) You must give any other recipients of the Work or
          Derivative Works a copy of this License; and

      (b) You must cause any modified files to carry prominent notices
          stating that You changed the files; and

      (c) You must retain, in the Source form of any Derivative Works
          that You distribute, all copyright, patent, trademark, and
          attribution notices from the Source form of the Work,
          excluding those notices that do not pertain to any part of
          the Derivative Works; and

      (d) If the Work includes a "NOTICE" text file as part of its
          distribution, then any Derivative Works that You distribute must
          include a readable copy of the attribution notices contained
          within such NOTICE file, excluding those notices that do not
          pertain to any part of the Derivative Works, in at least one
          of the following places: within a NOTICE text file distributed
          as part of the Derivative Works; within the Source form or
          documentation, if provided along with the Derivative Works; or,
          within a display generated by the Derivative Works, if and
          wherever such third-party notices normally appear. The contents
          of the NOTICE file are for informational purposes only and
          do not modify the License. You may add Your own attribution
          notices within Derivative Works that You distribute, alongside
          or as an addendum to the NOTICE text from the Work, provided
          that such additional attribution notices cannot be construed
          as modifying the License.

      You may add Your own copyright statement to Your modifications and
      may provide additional or different license terms and conditions
      for use, reproduction, or distribution of Your modifications, or
      for any such Derivative Works as a whole, provided Your use,
      reproduction, and distribution of the Work otherwise complies with
      the conditions stated in this License.

   5. Submission of Contributions. Unless You explicitly state otherwise,
      any Contribution intentionally submitted for inclusion in the Work
      by You to the Licensor shall be under the terms and conditions of
      this License, without any additional terms or conditions.
      Notwithstanding the above, nothing herein shall supersede or modify
      the terms of any separate license agreement you may have executed
      with Licensor regarding such Contributions.

   6. Trademarks. This License does not grant permission to use the trade
      names, trademarks, service marks, or product names of the Licensor,
      except as required for reasonable and customary use in describing the
      origin of the Work and reproducing the content of the NOTICE file.

   7. Disclaimer of Warranty. Unless required by applicable law or
      agreed to in writing, Licensor provides the Work (and each
      Contributor provides its Contributions) on an "AS IS" BASIS,
      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
      implied, including, without limitation, any warranties or conditions
      of TITLE, NON-INFRINGEMENT, MERCHANTABILITY, or FITNESS FOR A
      PARTICULAR PURPOSE. You are solely responsible for determining the
      appropriateness of using or redistributing the Work and assume any
      risks associated with Your exercise of permissions under this License.

   8. Limitation of Liability. In no event and under no legal theory,
      whether in tort (including negligence), contract, or otherwise,
      unless required by applicable law (such as deliberate and grossly
      negligent acts) or agreed to in writing, shall any Contributor be
      liable to You for damages, including any direct, indirect, special,
      incidental, or consequential damages of any character arising as a
      result of this License or out of the use or inability to use the
      Work (including but not limited to damages for loss of goodwill,
      work stoppage, computer failure or malfunction, or any and all
      other commercial damages or losses), even if such Contributor
      has been advised of the possibility of such damages.

   9. Accepting Warranty or Additional Liability. While redistributing
      the Work or Derivative Works thereof, You may choose to offer,
      and charge a fee for, acceptance of support, warranty, indemnity,
      or other liability obligations and/or rights consistent with this
      License. However, in accepting such obligations, You may act only
      on Your own behalf and on Your sole responsibility, not on behalf
      of any other Contributor, and only if You agree to indemnify,
      defend, and hold each Contributor harmless for any liability
      incurred by, or claims asserted against, such Contributor by reason
      of your accepting any such warranty or additional liability.

   END OF TERMS AND CONDITIONS

   APPENDIX: How to apply the Apache License to your work.

      To apply the Apache License to your work, attach the following
      boilerplate notice, with the fields enclosed by brackets "[]"
      replaced with your own identifying information. (Don't include
      the brackets!)  The text should be enclosed in the appropriate
      comment syntax for the file format. We also recommend that a
      file or class name and description of purpose be included on the
      same "printed page" as the copyright notice for easier
      identification within third-party archives.

   Copyright [yyyy] [name of copyright owner]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
```

</details>

<details><summary>BSD 3-Clause — MoveIt 자료와 저작권 고지</summary>

```text
BSD 3-Clause License

Copyright (c) 2008-2013, Willow Garage, Inc.
Copyright (c) 2015-2019, PickNik, LLC.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```

</details>
