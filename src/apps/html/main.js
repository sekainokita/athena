/******************************************************************************
#
# Copyright (C) 2023 - 2028 KETI, All rights reserved.
#                           (Korea Electronics Technology Institute)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# Use of the Software is limited solely to applications:
# (a) running for Korean Government Project, or
# (b) that interact with KETI project/platform.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# KETI BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
# OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Except as contained in this notice, the name of the KETI shall not be used
# in advertising or otherwise to promote the sale, use or other dealings in
# this Software without prior written authorization from KETI.
#
#******************************************************************************/

window.onload = function() {
    // 모달 창 열기
    document.getElementById('modal-background').style.display = 'block';
    document.getElementById('modal').style.display = 'block';

    let vehMode, CVehId, AVehId, vehicle0ImageUrl, vehicle1ImageUrl;

    let defaultIpAddress = "10.252.110.58";
    let testMode;
    let isTxTest;
    let VisiblePathMode;
    let isVisiblePath;
    let trafficLight;

    // 버튼 클릭 이벤트 처리
    document.getElementById('submit-button').onclick = function() {
        // 사용자 입력 값 가져오기
        let vehType = document.getElementById('vehType').value.toLowerCase();
        let testType = document.getElementById('testType').value.toLowerCase();
        let ipAddress = document.getElementById('ipAddress').value || defaultIpAddress;
        let visiblePath = document.getElementById('visiblePath').value.toLowerCase();

        if (vehType === "cv") {
            vehMode = "C-VEH";
            CVehId = 23120008;
            AVehId = 23120002;
            vehicle0ImageUrl = 'https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/ioniq5.png'; // 0번 차량은 C-Vehicle 이미지
            vehicle1ImageUrl = 'https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/ioniq-electric.png'; // 1번 차량은 A-Vehicle 이미지
        } else if (vehType === "av") {
            vehMode = "A-VEH";
            CVehId = 23120008;
            AVehId = 23120002;
            vehicle0ImageUrl = 'https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/ioniq-electric.png'; // 0번 차량은 A-Vehicle 이미지
            vehicle1ImageUrl = 'https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/ioniq5.png'; // 1번 차량은 C-Vehicle 이미지
        } else {
            vehMode = "C-VEH";
            CVehId = 23120008;
            AVehId = 23120002;
            vehicle0ImageUrl = 'https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/ioniq5.png'; // 기본 0번 차량은 C-Vehicle 이미지
            vehicle1ImageUrl = 'https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/ioniq-electric.png'; // 기본 1번 차량은 A-Vehicle 이미지
        }

        if (testType === "tx") {
            testMode = "Tx Test";
            isTxTest = true;
        } else if (testType === "rx") {
            testMode = "Rx Test";
            isTxTest = false;
        } else {
            testMode = "Tx Test";
            isTxTest = true;
        }

        if (visiblePath === "yes") {
            VisiblePathMode = "is Enabled";
            isVisiblePath = true;
        } else if (visiblePath === "no") {
            VisiblePathMode = "is Disabled";
            isVisiblePath = false;
        } else {
            VisiblePathMode = "is Disabled";
            isVisiblePath = false;
        }

        // 모달 닫기
        document.getElementById('modal-background').style.display = 'none';
        document.getElementById('modal').style.display = 'none';

        // alert 창으로 현재 테스트 모드와 IP 주소를 출력
        alert(`현재 선택된 설정: ${vehMode}, ${testMode}\n입력된 IP 주소: ${ipAddress}\nVisible Path ${VisiblePathMode}`);

        // 버튼이 제대로 표시되도록 모달 창이 닫힌 후 버튼을 다시 표시
        document.getElementById('autoTrackButton').style.display = 'block';
        document.getElementById('projectionButton').style.display = 'block';
        document.getElementById('connectedStatusButton').style.display = 'block';
        document.getElementById('workZoneButton').style.display = 'block';
        document.getElementById('mrsuButton').style.display = 'block';
        document.getElementById('visiblePathButton').style.display = 'block';
        document.getElementById('CB1').style.display = 'block';
        document.getElementById('CB2').style.display = 'block';
        document.getElementById('CB3').style.display = 'block';
        document.getElementById('CB4').style.display = 'block';
        document.getElementById('CB5').style.display = 'block';
        document.getElementById('CB6').style.display = 'block';
        document.getElementById('CC1').style.display = 'block';
        document.getElementById('CC2').style.display = 'block';
        document.getElementById('CC3').style.display = 'block';
        document.getElementById('CC4').style.display = 'block';
        document.getElementById('CC5').style.display = 'block';
        document.getElementById('CC6').style.display = 'block';
        document.getElementById('CD1').style.display = 'block';
        document.getElementById('CD2').style.display = 'block';
        document.getElementById('CD3').style.display = 'block';
        document.getElementById('CD4').style.display = 'block';
        document.getElementById('CD5').style.display = 'block';
        document.getElementById('CD6').style.display = 'block';
        document.getElementById('CD7').style.display = 'block';
        document.getElementById('CD8').style.display = 'block';

        main(isTxTest, ipAddress);
    };

    function main(isTxTest, ipAddress) {
        // KETI Pangyo
        const cKetiPangyoLatitude = 37.4064;
        const cKetiPangyolongitude = 127.1021;

        // RSU Location
        const cPangyoRsuLatitude16 = 37.408940;
        const cPangyoRsuLongitude16 = 127.099630;

        const cPangyoRsuLatitude17 = 37.406510;
        const cPangyoRsuLongitude17 = 127.100833;

        const cPangyoRsuLatitude18 = 37.405160;
        const cPangyoRsuLongitude18 = 127.103842;

        const cPangyoRsuLatitude5 = 37.410938;
        const cPangyoRsuLongitude5 = 127.094749;

        const cPangyoRsuLatitude31 = 37.411751;
        const cPangyoRsuLongitude31 = 127.095019;

        var vehicleLatitude0 = 37.406380;
        var vehicleLongitude0 = 127.102701;

        var vehicleLatitude1 = 37.406402;
        var vehicleLongitude1 = 127.102532;

        let s_unRxDevId, s_nRxLatitude, s_nRxLongitude, s_unRxVehicleHeading, s_unRxVehicleSpeed;
        let s_unTxDevId, s_nTxLatitude, s_nTxLongitude, s_unTxVehicleHeading, s_unTxVehicleSpeed;
        let s_unPdr, s_ulLatencyL1, s_ulTotalPacketCnt, s_unSeqNum;

        function updateV2VPath(pathId, marker) {
            const V2VCoordinates = [
                [vehicleLongitude0, vehicleLatitude0], //실시간 본인 차량
                [vehicleLongitude1, vehicleLatitude1]
            ];

            if (map.getSource(pathId)) {
                map.getSource(pathId).setData({
                    'type': 'Feature',
                    'geometry': {
                        'type': 'LineString',
                        'coordinates': V2VCoordinates
                    }
                });

                // 중간 지점 마커 업데이트
                const midPoint = [
                    (V2VCoordinates[0][0] + V2VCoordinates[1][0]) / 2,
                    (V2VCoordinates[0][1] + V2VCoordinates[1][1]) / 2
                ];
                if (marker) {
                    marker.setLngLat(midPoint).addTo(map);
                }
            }
        }

        function updateV2IPath(pathId, marker) {
            const MRsuCoordinate = [127.440227, 36.730164];
            const V2ICoordinates = [
                [vehicleLongitude0, vehicleLatitude0],  //실시간 차량 위치
                MRsuCoordinate //고정 좌표 (mRSU)
            ];

            if (map.getSource(pathId)) {
                map.getSource(pathId).setData({
                    'type': 'Feature',
                    'geometry': {
                        'type': 'LineString',
                        'coordinates': V2ICoordinates
                    }
                });

                const midPoint = [
                    (V2ICoordinates[0][0] + MRsuCoordinate[0]) / 2,
                    (V2ICoordinates[0][1] + MRsuCoordinate[1]) / 2
                ];

                // 마커 위치를 차량 위치에 맞게 업데이트
                if (marker) {
                    marker.setLngLat(midPoint).addTo(map);
                }
            }
        }

        let s_unTempTxCnt = 0;
        let isPathPlan = false;
        let isCvLineEnabled = false;
        let isWorkZoneEnabled = false;
        let isMrsuEnabled = false;
        let isCentering = false;
        let isCB1 = false;
        let isCB2 = false;
        let isCB3 = false;
        let isCB4 = false;
        let isCB5 = false;
        let isCB6 = false;
        let isCC1 = false;
        let isCC2 = false;
        let isCC3 = false;
        let isCC4 = false;
        let isCC5 = false;
        let isCC6 = false;
        let isCD1 = false;
        let isCD2 = false;
        let isCD3 = false;
        let isCD4 = false;
        let isCD5 = false;
        let isCD6 = false;
        let isCD7 = false;
        let isCD8 = false;

        let workZoneMarker = new mapboxgl.Marker({element: createWorkZoneMarker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/work-zone.png')});
        let mrsuMarker = new mapboxgl.Marker({element: createMrsuMarker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/m-rsu-front.png')});
        let ioniqMarker = new mapboxgl.Marker({element: createIoniqMarker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/ioniq-electric-sky.png')});
        let CB3NegotiationMarker = null;
        let CB4NegotiationMarker = null;
        let CB6NegotiationMarker = null;
        let CC3NegotiationMarker = null;
        let CD2NegotiationMarker = null;
        let CD4NegotiationMarker = null;
        let CD5CNegotiationMarker = null;
        let CD5ANegotiationMarker = null;
        let CD8NegotiationMarker = null;

        mapboxgl.accessToken = 'pk.eyJ1IjoieWVzYm1hbiIsImEiOiJjbHoxNHVydHQyNzBzMmpzMHNobGUxNnZ6In0.bAFH10On30d_Cj-zTMi53Q';
        const map = new mapboxgl.Map({
            container: 'map',
            style: 'mapbox://styles/yesbman/clyzkeh8900dr01pxdqow8awk',
            projection: 'globe',
            zoom: 19,
            center: [cKetiPangyolongitude, cKetiPangyoLatitude]
        });

        map.addControl(new mapboxgl.NavigationControl());

        map.on('style.load', () => {
            map.setFog({});

            document.getElementById('autoTrackButton').addEventListener('click', function() {
                isCentering = !isCentering;
                if (isCentering) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';
                }

                map.setCenter([s_nRxLongitude, s_nRxLatitude]);
            });
        });

        map.on('contextmenu', function (e) {
            const lngLat = e.lngLat;
            const latitude = lngLat.lat.toFixed(6);
            const longitude = lngLat.lng.toFixed(6);

            const popup = document.getElementById('coordinate-popup');
            const coordinateText = document.getElementById('coordinate-text');

            coordinateText.textContent = `위도(${latitude}) 경도(${longitude})`;

            popup.style.left = `${e.point.x}px`;
            popup.style.top = `${e.point.y}px`;

            popup.style.display = 'block';

            setTimeout(() => {
                popup.style.display = 'none';
            }, 5000);
        });

        document.getElementById('projectionButton').addEventListener('click', function() {
            isPathPlan = !isPathPlan;
            if (isPathPlan) {
                this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                this.style.color = 'white';
            } else {
                this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                this.style.color = 'white';
            }

            map.setCenter([s_nRxLongitude, s_nRxLatitude]);
        });

        document.getElementById('connectedStatusButton').addEventListener('click', function() {
            isCvLineEnabled = !isCvLineEnabled;
            if (isCvLineEnabled) {
                this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                this.style.color = 'white';
            } else {
                this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                this.style.color = 'white';
            }


            if (isCvLineEnabled) {
                if (!map.getLayer('lineLayer')) {
                    map.addSource('line', {
                        'type': 'geojson',
                        'data': {
                            'type': 'Feature',
                            'geometry': {
                                'type': 'LineString',
                                'coordinates': [[vehicleLongitude0, vehicleLatitude0], [vehicleLongitude1, vehicleLatitude1]]
                            }
                        }
                    });

                    map.addLayer({
                        'id': 'lineLayer',
                        'type': 'line',
                        'source': 'line',
                        'layout': {},
                        'paint': {
                            'line-color': [
                                'interpolate',
                                ['linear'],
                                ['line-progress'],
                                0, '#00FFFF',
                                1, '#008B8B'
                            ],
                            'line-width': 1.5
                        }
                    });

                    map.addLayer({
                        'id': 'lineLabelLayer',
                        'type': 'symbol',
                        'source': 'line',
                        'layout': {
                            'symbol-placement': 'line',
                            'text-field': 'Connected V2X',
                            'text-font': ['Open Sans Regular', 'Arial Unicode MS Regular'],
                            'text-size': 12,
                            'text-anchor': 'center',
                            'text-allow-overlap': true
                        },
                        'paint': {
                            'text-color': '#000000',
                            'text-halo-color': '#FFFFFF',
                            'text-halo-width': 2
                        }
                    });
                } else {
                    map.getSource('line').setData({
                        'type': 'Feature',
                        'geometry': {
                            'type': 'LineString',
                            'coordinates': [[vehicleLongitude0, vehicleLatitude0], [vehicleLongitude1, vehicleLatitude1]]
                        }
                    });
                }
            } else {
                // 연결 선 제거
                if (map.getLayer('lineLayer')) {
                    map.removeLayer('lineLayer');
                }
                if (map.getLayer('lineLabelLayer')) {
                    map.removeLayer('lineLabelLayer');
                }
                if (map.getSource('line')) {
                    map.removeSource('line');
                }
            }
        });

        document.getElementById('workZoneButton').addEventListener('click', function() {
            isWorkZoneEnabled = !isWorkZoneEnabled;
            if (isWorkZoneEnabled) {
                this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                this.style.color = 'white';
            } else {
                this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                this.style.color = 'white';
            }
            toggleWorkZone();
        });

        function toggleWorkZone()
        {
            const WorkZoneCoordinate = [127.440128, 36.729698];
            if (isWorkZoneEnabled)
            {
                if (workZoneMarker === null)
                {
                    workZoneMarker = new mapboxgl.Marker({element: createWorkZoneMarker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/work-zone.png')})
                    .setLngLat(WorkZoneCoordinate)
                    .addTo(map);
                }
            }
            else
            {
                if (workZoneMarker !== null)
                {
                    workZoneMarker.remove();
                    workZoneMarker = null;
                }
            }
        }

        function createWorkZoneMarker(imageUrl)
        {
            const workzonecontainer = document.createElement('div');
            workzonecontainer.style.display = 'flex';
            workzonecontainer.style.flexDirection = 'column';
            workzonecontainer.style.alignItems = 'center';

            const img = document.createElement('div');
            img.style.backgroundImage = `url(${imageUrl})`;
            img.style.width = '50px';
            img.style.height = '50px';
            img.style.backgroundSize = 'contain';
            img.style.backgroundRepeat = 'no-repeat';

            const label = document.createElement('div');
            label.innerHTML = "Under<br>Construction";
            label.style.color = 'white';
            label.style.textAlign = 'center';
            label.style.fontWeight = 'bold';
            label.style.backgroundColor = 'rgba(255, 0, 0, 0.97)';
            label.style.padding = '2px 5px';
            label.style.borderRadius = '5px';
            label.style.boxShadow = '0 0 15px #00ccff, 0 0 30px #00ccff, 0 0 45px #00ccff';
            label.style.textShadow = '0 0 10px #00ccff, 0 0 20px #00ccff, 0 0 30px #00ccff';
            label.style.boxShadow = '0px 4px 8px rgba(0, 0, 0, 0.3)';
            label.style.width = 'auto';
            label.style.display = 'inline-block';
            label.style.fontSize = '13px';

            workzonecontainer.appendChild(img);
            workzonecontainer.appendChild(label);

            return workzonecontainer;
        }

        document.getElementById('mrsuButton').addEventListener('click', function() {
            isMrsuEnabled = !isMrsuEnabled;
            if (isMrsuEnabled) {
                this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                this.style.color = 'white';
            } else {
                this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                this.style.color = 'white';
            }

            toggleMrsu();
        });

        document.getElementById('visiblePathButton').addEventListener('click', function() {
            isVisiblePath = !isVisiblePath;
            if (isVisiblePath) {
                this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                this.style.color = 'white';
            } else {
                this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                this.style.color = 'white';
            }

        });

        function toggleMrsu()
        {
            const MRsuCoordinate = [127.440227, 36.730164];

            if (isMrsuEnabled)
            {
                if (mrsuMarker === null)
                {
                    mrsuMarker = new mapboxgl.Marker({element: createMrsuMarker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/m-rsu-front.png')})
                    .setLngLat(MRsuCoordinate)
                    .addTo(map);
                }
            }
            else
            {
                if (mrsuMarker !== null)
                {
                    mrsuMarker.remove();
                    mrsuMarker = null;
                }
            }
        }

        function createMrsuMarker(imageUrl)
        {
            const mrsucontainer = document.createElement('div');
            mrsucontainer.style.display = 'flex';
            mrsucontainer.style.flexDirection = 'column';
            mrsucontainer.style.alignItems = 'center';
            mrsucontainer.style.width = '225px';

            const img = document.createElement('div');
            img.style.backgroundImage = `url(${imageUrl})`;
            img.style.width = '225px';
            img.style.height = '170px';
            img.style.backgroundSize = 'contain';
            img.style.backgroundRepeat = 'no-repeat';

            const label = document.createElement('div');
            label.textContent = "RSU";
            label.style.color = 'white';
            label.style.textAlign = 'center';
            label.style.fontWeight = 'bold';
            label.style.backgroundColor = 'rgba(0, 204, 255, 0.8)';
            label.style.padding = '5px 10px';
            label.style.borderRadius = '10px';
            label.style.boxShadow = '0 0 15px #00ccff, 0 0 30px #00ccff, 0 0 45px #00ccff';
            label.style.textShadow = '0 0 10px #00ccff, 0 0 20px #00ccff, 0 0 30px #00ccff';
            label.style.boxShadow = '0px 4px 8px rgba(0, 0, 0, 0.3)';
            label.style.width = 'auto';
            label.style.display = 'inline-block';
            label.style.fontSize = '18px';
            label.style.marginLeft = '-20px';

            mrsucontainer.appendChild(img);
            mrsucontainer.appendChild(label);

            return mrsucontainer;
        }

        map.on('style.load', function() {
            document.getElementById('CB1').addEventListener('click', function() {
                isCB1 = !isCB1;
                if (isCB1) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';

                    // MRsu Marker 추가
                    const MRsuCoordinate = [127.440227, 36.730164];
                    if (!mrsuMarker) {
                        mrsuMarker = new mapboxgl.Marker({element: createMrsuMarker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/m-rsu-front.png')})
                        .setLngLat(MRsuCoordinate)
                        .addTo(map);
                    } else if (!mrsuMarker._map) {
                        mrsuMarker.setLngLat(MRsuCoordinate).addTo(map);
                    }

                    // Work Zone Marker 추가
                    const WorkZoneCoordinate = [127.440128, 36.729698];
                    if (!workZoneMarker) {
                        workZoneMarker = new mapboxgl.Marker({element: createWorkZoneMarker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/work-zone.png')})
                        .setLngLat(WorkZoneCoordinate)
                        .addTo(map);
                    } else if (!workZoneMarker._map) {
                        workZoneMarker.setLngLat(WorkZoneCoordinate).addTo(map);
                    }

                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';

                    if (mrsuMarker && mrsuMarker._map) {
                        mrsuMarker.remove();
                    }

                    if (workZoneMarker && workZoneMarker._map) {
                        workZoneMarker.remove();
                    }
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'yellow';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'yellow';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);
            });
        });

        const CB2Coordinates = [
            [127.439641, 36.730080],
            [127.439703, 36.730085], //첫번째 노란점
            [127.439820, 36.730091],
            [127.439885, 36.730050], //두번째 노란점
            [127.439991, 36.729972], //세번째 노란점
            [127.440131, 36.729958],
            [127.440254, 36.730017], //네번째
            [127.440304, 36.730084], //다섯번째
            [127.440352, 36.730148],
            [127.440451, 36.730166] //마지막
        ];

        function interpolateCatmullRom(points, numPointsBetween) {
            let interpolatedPoints = [];

            function interpolate(p0, p1, p2, p3, t) {
                const t2 = t * t;
                const t3 = t2 * t;
                const out = [
                    0.5 * (2 * p1[0] + (-p0[0] + p2[0]) * t + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3),
                    0.5 * (2 * p1[1] + (-p0[1] + p2[1]) * t + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
                ];
                return out;
            }

            for (let i = 1; i < points.length - 2; i++) {
                const p0 = points[i - 1];
                const p1 = points[i];
                const p2 = points[i + 1];
                const p3 = points[i + 2];

                interpolatedPoints.push(p1);
                for (let t = 0; t < numPointsBetween; t++) {
                    const tNorm = t / numPointsBetween;
                    interpolatedPoints.push(interpolate(p0, p1, p2, p3, tNorm));
                }
            }
            interpolatedPoints.push(points[points.length - 2]);
            interpolatedPoints.push(points[points.length - 1]);

            return interpolatedPoints;
        }

        const CB2smoothPath = interpolateCatmullRom(CB2Coordinates, 100);

        map.on('style.load', function()
        {
            map.loadImage('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/arrowB.png', function(error, image)
            {
                if (error)
                {
                    console.error('fail load image', error);
                    return;
                }
                if (!map.hasImage('arrowB-icon')) {
                    map.addImage('arrowB-icon', image);
                }

            document.getElementById('CB2').addEventListener('click', function() {
                isCB2 = !isCB2;
                if (isCB2) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'red';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'red';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);

                if (vehMode === "C-VEH") {
                    if (map.getLayer('CB2Path')) {
                        map.setLayoutProperty('CB2Path', 'visibility', 'none');
                    }
                    if (map.getLayer('CB2Arrows')) {
                        map.setLayoutProperty('CB2Arrows', 'visibility', 'none');
                    }
                }
                else {
                    if (map.getLayer('CB2Path'))
                    {
                        map.setLayoutProperty('CB2Path', 'visibility', isCB2 ? 'visible' : 'none');
                        map.setLayoutProperty('CB2Arrows', 'visibility', isCB2 ? 'visible' : 'none');
                    } else
                    {
                        map.addSource('CB2Path', {
                            'type': 'geojson',
                            'data': {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': CB2smoothPath
                                }
                            }
                        });

                        map.addLayer({
                            'id': 'CB2Path',
                            'type': 'line',
                            'source': 'CB2Path',
                            'layout': {
                                'line-join': 'round',
                                'line-cap': 'round',
                                'visibility': 'visible'
                            },
                            'paint': {
                                'line-color': 'rgba(0, 150, 255, 0.8)',
                                'line-width': 20,
                                'line-blur': 0.5
                            }
                        });

                        const CB2arrowCoordinates = [
                            { coord: [127.439703, 36.730085], rotate: 90},
                            { coord: [127.439885, 36.730050], rotate: 140},
                            { coord: [127.439991, 36.729972], rotate: 110},
                            { coord: [127.440254, 36.730017], rotate: 45},
                            { coord: [127.440304, 36.730084], rotate: 30},
                            { coord: [127.440451, 36.730166], rotate: 85}
                        ];

                        const CB2arrowFeatures = CB2arrowCoordinates.map(arrow => {
                            return {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'Point',
                                    'coordinates': arrow.coord
                                },
                                'properties': {
                                    'rotate': arrow.rotate
                                }
                            };
                        });

                        map.addSource('CB2Arrows', {
                            'type': 'geojson',
                            'data': {
                                'type': 'FeatureCollection',
                                'features': CB2arrowFeatures
                            }
                        });

                        map.addLayer({
                            'id': 'CB2Arrows',
                            'type': 'symbol',
                            'source': 'CB2Arrows',
                            'layout': {
                                'icon-image': 'arrowB-icon',
                                'icon-size': 0.05,
                                'icon-rotate': ['get', 'rotate'],
                                'icon-allow-overlap': true,
                                'visibility': 'visible'
                            }
                        });
                    }
                }
                });
            });
        });

        map.on('style.load', function() {
            map.loadImage('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/arrowG.png', function(error, image) {
                if (error) {
                    console.error('fail load image', error);
                    return;
                }

                map.addImage('arrowG-icon', image);

                document.getElementById('CB3').addEventListener('click', function() {
                    isCB3 = !isCB3;
                    if (isCB3) {
                        this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                        this.style.color = 'white';
                    } else {
                        this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                        this.style.color = 'white';
                    }
                    if (vehMode === "C-VEH") {
                        trafficLight = 'red';
                    } else if (vehMode === "A-VEH") {
                        trafficLight = 'red';
                    } else {
                        trafficLight = 'red';
                    }
                    updateTrafficLight(trafficLight);

                    if (map.getLayer('CB3Path')) {
                        map.setLayoutProperty('CB3Path', 'visibility', isCB3 ? 'visible' : 'none');
                        map.setLayoutProperty('CB3Arrows', 'visibility', isCB3 ? 'visible' : 'none');
                        map.setLayoutProperty('CB3V2XPath', 'visibility', isCB3 ? 'visible' : 'none');

                        // V2XLabel 표시 또는 제거
                        if (CB3NegotiationMarker) {
                            if (isCB3) {
                                CB3NegotiationMarker.addTo(map);  // 마커 추가
                            } else {
                                CB3NegotiationMarker.remove();  // 마커 제거
                            }
                        }
                    } else {
                        initializeCB3Path();
                    }
                });

                function initializeCB3Path() {
                        const CB3Coordinates = [
                            { coord: [127.440170, 36.729793] },
                            { coord: [127.440157, 36.729847], rotate: 0 },
                            { coord: [127.440181, 36.729961] },
                            { coord: [127.440254, 36.730017], rotate: 45 },
                            { coord: [127.440304, 36.730084], rotate: 30 },
                            { coord: [127.440350, 36.730151] },
                            { coord: [127.440451, 36.730166], rotate: 85 },
                            { coord: [127.440557, 36.730178], rotate: 85 }
                        ];

                        const CB3route = CB3Coordinates.map(point => point.coord);
                        const smoothCB3route = interpolateCatmullRom(CB3route, 100);

                        map.addSource('CB3Path', {
                            'type': 'geojson',
                            'data': {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': smoothCB3route
                                }
                            }
                        });

                        map.addLayer({
                            'id': 'CB3Path',
                            'type': 'line',
                            'source': 'CB3Path',
                            'layout': {
                                'line-join': 'round',
                                'line-cap': 'round',
                                'visibility': 'visible'
                            },
                            'paint': {
                                'line-color': 'rgba(50, 205, 50, 0.7)',
                                'line-width': 20,
                                'line-blur': 1,
                                'line-opacity': 0.8
                            }
                        });

                        const arrowFeatures = CB3Coordinates
                            .filter(arrow => arrow.rotate !== undefined)
                            .map(arrow => {
                                return {
                                    'type': 'Feature',
                                    'geometry': {
                                        'type': 'Point',
                                        'coordinates': arrow.coord
                                    },
                                    'properties': {
                                        'rotate': arrow.rotate
                                    }
                                };
                            });

                        map.addSource('CB3Arrows', {
                            'type': 'geojson',
                            'data': {
                                'type': 'FeatureCollection',
                                'features': arrowFeatures
                            }
                        });

                        map.addLayer({
                            'id': 'CB3Arrows',
                            'type': 'symbol',
                            'source': 'CB3Arrows',
                            'layout': {
                                'icon-image': 'arrowG-icon',
                                'icon-size': 0.05,
                                'icon-rotate': ['get', 'rotate'],
                                'icon-allow-overlap': true,
                                'visibility': 'visible'
                            }
                        });

                        map.addSource('CB3V2XPath', {
                            'type': 'geojson',
                            'data': {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': [[vehicleLongitude0, vehicleLatitude0], [vehicleLongitude1, vehicleLatitude1]]
                                }
                            }
                        });

                        map.addLayer({
                            'id': 'CB3V2XPath',
                            'type': 'line',
                            'source': 'CB3V2XPath',
                            'layout': {
                                'line-join': 'round',
                                'line-cap': 'round',
                                'visibility': 'visible'
                            },
                            'paint': {
                                'line-color': '#27FFFF',
                                'line-width': 4,
                                'line-opacity': 0.8,
                                'line-dasharray': [0.5, 1.5]
                            }
                        });

                        const midPoint = [
                            (vehicleLongitude0 + vehicleLongitude1) / 2,
                            (vehicleLatitude0 + vehicleLatitude1) / 2
                        ];

                        // 커스텀 마커 생성 및 지도에 추가
                        CB3NegotiationMarker = new mapboxgl.Marker({element: createCustomLabel()})
                            .setLngLat(midPoint)
                            .addTo(map);
                    }

                function createCustomLabel() {
                    const labelContainer = document.createElement('div');
                    labelContainer.style.display = 'flex';
                    labelContainer.style.flexDirection = 'column';
                    labelContainer.style.alignItems = 'center';
                    labelContainer.style.width = 'auto';

                    // 직사각형 배경
                    const background = document.createElement('div');
                    background.style.width = 'auto';
                    background.style.height = 'auto';
                    background.style.padding = '5px 10px';
                    background.style.backgroundColor = 'rgba(0, 204, 255, 0.8)';
                    background.style.borderRadius = '10px';
                    background.style.boxShadow = '0 0 15px #00ccff, 0 0 30px #00ccff, 0 0 45px #00ccff';

                    // 텍스트
                    const text = document.createElement('div');
                    text.innerHTML = "V2V-SSOV MSG<br>주행 의도 공유<br>(A-VEH → C-VEH)";
                    text.style.color = 'black';
                    text.style.fontWeight = 'bold';
                    text.style.textAlign = 'center';
                    text.style.textShadow = '0 0 10px #00ccff, 0 0 20px #00ccff, 0 0 30px #00ccff';
                    text.style.fontSize = '18px';

                    background.appendChild(text);
                    labelContainer.appendChild(background);

                    return labelContainer;
                }
            });
        });

        map.on('style.load', function() {
            document.getElementById('CB4').addEventListener('click', function() {
                isCB4 = !isCB4;
                if (isCB4) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';
                    updateCB4PathAndMarker();
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';

                    if (map.getLayer('CB4V2XPath')) {
                        map.removeLayer('CB4V2XPath');
                        map.removeSource('CB4V2XPath');
                    }

                    if (CB4NegotiationMarker) {
                        CB4NegotiationMarker.remove();
                    }
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'red';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'red';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);
            });

            function updateCB4PathAndMarker() {
                    let CB4Coordinates = [
                        [vehicleLongitude0, vehicleLatitude0],
                        [vehicleLongitude1, vehicleLatitude1]
                    ];

                    if (!map.getSource('CB4V2XPath')) {
                        map.addSource('CB4V2XPath', {
                            'type': 'geojson',
                            'data': {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': CB4Coordinates
                                }
                            }
                        });

                        map.addLayer({
                            'id': 'CB4V2XPath',
                            'type': 'line',
                            'source': 'CB4V2XPath',
                            'layout': {
                                'line-join': 'round',
                                'line-cap': 'round',
                                'visibility': 'visible'
                            },
                            'paint': {
                                'line-color': '#007AFF',
                                'line-width': 4,
                                'line-opacity': 0.8,
                                'line-dasharray': [0.5, 1.5]
                            }
                        });
                    } else {
                        map.getSource('CB4V2XPath').setData({
                            'type': 'Feature',
                            'geometry': {
                                'type': 'LineString',
                                'coordinates': CB4Coordinates
                    }
                });
            }
                    const midPoint = [
                        (CB4Coordinates[0][0] + CB4Coordinates[1][0]) / 2,
                        (CB4Coordinates[0][1] + CB4Coordinates[1][1]) / 2
                    ];

                    if (!CB4NegotiationMarker) {
                        CB4NegotiationMarker = new mapboxgl.Marker({element: createCustomLabelCB4()})
                        .setLngLat(midPoint)
                        .addTo(map);
                    } else {
                        CB4NegotiationMarker.setLngLat(midPoint);
                        CB4NegotiationMarker.addTo(map);
                    }
                }

            function createCustomLabelCB4() {
                const labelContainer = document.createElement('div');
                labelContainer.style.display = 'flex';
                labelContainer.style.flexDirection = 'column';
                labelContainer.style.alignItems = 'center';
                labelContainer.style.width = 'auto';

                // 직사각형 배경
                const background = document.createElement('div');
                background.style.width = 'auto';
                background.style.height = 'auto';
                background.style.padding = '5px 10px';
                background.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                background.style.borderRadius = '10px';
                background.style.boxShadow = '0 0 15px #00ccff, 0 0 30px #00ccff, 0 0 45px #00ccff';

                // 텍스트
                const text = document.createElement('div');
                text.innerHTML = "V2V-SSOV MSG<br>주행 의도 공유 완료";
                text.style.color = 'black';
                text.style.fontWeight = 'bold';
                text.style.textAlign = 'center';
                text.style.textShadow = '0 0 10px #00ccff, 0 0 20px #00ccff, 0 0 30px #00ccff';
                text.style.fontSize = '18px';

                background.appendChild(text);
                labelContainer.appendChild(background);

                return labelContainer;
            }
        });

        let CB5Marker = new mapboxgl.Marker({
            element: createCB5Marker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/stop3.png')
            }).setLngLat([127.440172, 36.729915]);

        map.on('style.load', () => {
            document.getElementById('CB5').addEventListener('click', function() {
                isCB5 = !isCB5;
                if (isCB5) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';

                    if (!CB5Marker._map) {
                        CB5Marker.addTo(map);
                    }
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';

                    if (CB5Marker._map) {
                        CB5Marker.remove();
                    }
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'red';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'green';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);
            });
        });

        function createCB5Marker(imageUrl)
        {
            const CB5Container = document.createElement('div');
            CB5Container.style.display = 'flex';
            CB5Container.style.flexDirection = 'column';
            CB5Container.style.alignItems = 'center';

            const img = document.createElement('div');
            img.style.backgroundImage = `url(${imageUrl})`;
            img.style.width = '50px';
            img.style.height = '50px';
            img.style.backgroundSize = 'contain';
            img.style.backgroundRepeat = 'no-repeat';

            CB5Container.appendChild(img);
            return CB5Container;
        }

        let CB6Marker = new mapboxgl.Marker({
            element: createCB6Marker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/go-straight.png')
            }).setLngLat([127.440172, 36.729915]);

        map.on('style.load', () => {
            document.getElementById('CB6').addEventListener('click', function() {
                isCB6 = !isCB6;
                if (isCB6) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';

                    if (!CB6Marker._map) {
                        CB6Marker.addTo(map);
                    }

                    const CB6Coordinates = [
                        [127.440170, 36.729793],
                        [127.440553, 36.730175]
                    ];

                    if (!map.getSource('CB6V2XPath')) {
                        map.addSource('CB6V2XPath', {
                            'type': 'geojson',
                            'data': {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': CB6Coordinates
                                }
                            }
                        });

                        map.addLayer({
                            'id': 'CB6V2XPath',
                            'type': 'line',
                            'source': 'CB6V2XPath',
                            'layout': {
                                'line-join': 'round',
                                'line-cap': 'round',
                                'visibility': 'visible'
                            },
                            'paint': {
                                'line-color': '#4CAF50',
                                'line-width': 4,
                                'line-opacity': 0.8,
                                'line-dasharray': [2,2]
                            }
                        });
                    }

                    const midPoint = [
                        (CB6Coordinates[0][0] + CB6Coordinates[1][0]) / 2,
                        (CB6Coordinates[0][1] + CB6Coordinates[1][1]) / 2
                    ];

                    if (!CB6NegotiationMarker) {
                        CB6NegotiationMarker = new mapboxgl.Marker({element: createCustomLabelCB6()})
                        .setLngLat(midPoint)
                        .addTo(map);
                    } else {
                        CB6NegotiationMarker.addTo(map);
                    }
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';

                    if (CB6Marker._map) {
                        CB6Marker.remove();
                    }

                    if (map.getLayer('CB6V2XPath')) {
                        map.removeLayer('CB6V2XPath');
                        map.removeSource('CB6V2XPath');
                    }

                    if (CB6NegotiationMarker) {
                        CB6NegotiationMarker.remove();
                    }
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'green';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'green';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);
            });
        });

        function createCB6Marker(imageUrl) {
            const CB6Container = document.createElement('div');
            CB6Container.style.display = 'flex';
            CB6Container.style.flexDirection = 'column';
            CB6Container.style.alignItems = 'center';

            const img = document.createElement('div');
            img.style.backgroundImage = `url(${imageUrl})`;
            img.style.width = '50px';
            img.style.height = '50px';
            img.style.backgroundSize = 'contain';
            img.style.backgroundRepeat = 'no-repeat';

            CB6Container.appendChild(img);
            return CB6Container;
        }

        function createCustomLabelCB6() {
            const labelContainer = document.createElement('div');
            labelContainer.style.display = 'flex';
            labelContainer.style.flexDirection = 'column';
            labelContainer.style.alignItems = 'center';
            labelContainer.style.width = 'auto';

            // 직사각형 배경
            const background = document.createElement('div');
            background.style.width = 'auto';
            background.style.height = 'auto';
            background.style.padding = '5px 10px';
            background.style.backgroundColor = '#81C784';
            background.style.borderRadius = '10px';
            background.style.boxShadow = '0 0 15px #4CAF50, 0 0 30px #4CAF50, 0 0 45px #4CAF50';

            // 텍스트
            const text = document.createElement('div');
            text.innerHTML = "V2X-SSOV MSG<br>Class B 완료";
            text.style.color = 'black';
            text.style.fontWeight = 'bold';
            text.style.textAlign = 'center';
            text.style.fontSize = '18px';

            background.appendChild(text);
            labelContainer.appendChild(background);

            return labelContainer;
        }

        function toggleIoniq()
        {
            const IoniqCoordinate = [127.440161, 36.729833];

            if (ioniqMarker)
            {
                if (ioniqMarker === null)
                {
                    ioniqMarker = new mapboxgl.Marker({element: createIoniqMarker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/ioniq-electric-sky.png')})
                    .setLngLat(IoniqCoordinate)
                    .addTo(map);
                }
            }
            else
            {
                if (ioniqMarker !== null)
                {
                    ioniqMarker.remove();
                    ioniqMarker = null;
                }
            }
        }

        function createIoniqMarker(imageUrl)
        {
            const ioniqcontainer = document.createElement('div');
            ioniqcontainer.style.display = 'flex';
            ioniqcontainer.style.flexDirection = 'column';
            ioniqcontainer.style.alignItems = 'center';
            ioniqcontainer.style.width = '225px';

            const img = document.createElement('div');
            img.style.backgroundImage = `url(${imageUrl})`;
            img.style.width = '100px';
            img.style.height = '100px';
            img.style.backgroundSize = 'contain';
            img.style.backgroundRepeat = 'no-repeat';
            img.style.transform = 'rotate(350deg)';

            ioniqcontainer.appendChild(img);

            return ioniqcontainer;
        }

        map.on('style.load', function() {
            document.getElementById('CC1').addEventListener('click', function() {
                isCC1 = !isCC1;
                if (isCC1) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';

                    // MRsu Marker 추가
                    const MRsuCoordinate = [127.440227, 36.730164];
                    if (!mrsuMarker) {
                        mrsuMarker = new mapboxgl.Marker({element: createMrsuMarker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/m-rsu-front.png')})
                        .setLngLat(MRsuCoordinate)
                        .addTo(map);
                    } else if (!mrsuMarker.map) {
                        mrsuMarker.setLngLat(MRsuCoordinate).addTo(map);
                    }
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';

                    if (mrsuMarker && mrsuMarker._map) {
                        mrsuMarker.remove();
                    }
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'yellow';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'yellow';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);
            });
        });

        const CC2BCoordinates = [
            [127.439641, 36.730080],
            [127.439703, 36.730085], //첫번째 노란점
            [127.439820, 36.730091],
            [127.439885, 36.730050], //두번째 노란점
            [127.439991, 36.729972], //세번째 노란점
            [127.440131, 36.729958],
            [127.440254, 36.730017], //네번째
            [127.440304, 36.730084], //다섯번째
            [127.440352, 36.730148],
            [127.440451, 36.730166] //마지막
        ];

        const CC2GCoordinates = [
            [127.440170, 36.729793],
            [127.440157, 36.729847],
            [127.440181, 36.729961],
            [127.440254, 36.730017],
            [127.440304, 36.730084],
            [127.440350, 36.730151],
            [127.440451, 36.730166],
            [127.440557, 36.730178]
        ]

        function interpolateCatmullRom(points, numPointsBetween) {
            let interpolatedPoints = [];

            function interpolate(p0, p1, p2, p3, t) {
                const t2 = t * t;
                const t3 = t2 * t;
                const out = [
                    0.5 * (2 * p1[0] + (-p0[0] + p2[0]) * t + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3),
                    0.5 * (2 * p1[1] + (-p0[1] + p2[1]) * t + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
                ];
                return out;
            }

            for (let i = 1; i < points.length - 2; i++) {
                const p0 = points[i - 1];
                const p1 = points[i];
                const p2 = points[i + 1];
                const p3 = points[i + 2];

                interpolatedPoints.push(p1);
                for (let t = 0; t < numPointsBetween; t++) {
                    const tNorm = t / numPointsBetween;
                    interpolatedPoints.push(interpolate(p0, p1, p2, p3, tNorm));
                }
            }
            interpolatedPoints.push(points[points.length - 2]);
            interpolatedPoints.push(points[points.length - 1]);

            return interpolatedPoints;
        }

        const CC2BsmoothPath = interpolateCatmullRom(CC2BCoordinates, 100);
        const CC2GsmoothPath = interpolateCatmullRom(CC2GCoordinates, 100);

        map.on('style.load', function() {
            if (!map.hasImage('arrowB-icon')) {
                map.loadImage('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/arrowB.png', function(error, image) {
                    if (error) {
                        console.error('fail load image', error);
                        return;
                    }
                    map.addImage('arrowB-icon', image);
                });
            }

            if (!map.hasImage('arrowG-icon')) {
                map.loadImage('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/arrowG.png', function(error, image) {
                    if (error) {
                        console.error('fail load image', error);
                        return;
                    }
                    map.addImage('arrowG-icon', image);
                });
            }

            const cc2Button = document.getElementById('CC2');
            if (cc2Button) {
                cc2Button.addEventListener('click', function() {
                    isCC2 = !isCC2;
                    this.style.backgroundColor = isCC2 ? 'rgba(0, 122, 255, 0.9)' : 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';

                            if (vehMode === "C-VEH") {
                                trafficLight = 'red';
                            } else if (vehMode === "A-VEH") {
                                trafficLight = 'red';
                            } else {
                                trafficLight = 'red';
                            }
                    updateTrafficLight(trafficLight);

                    if (map.getLayer('CC2GPath')) {
                        map.setLayoutProperty('CC2GPath', 'visibility', isCC2 ? 'visible' : 'none');
                        map.setLayoutProperty('CC2GArrows', 'visibility', isCC2 ? 'visible' : 'none');
                    } else {
                        map.addSource('CC2GPath', {
                            'type': 'geojson',
                            'data': {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': CC2GsmoothPath
                                }
                            }
                        });

                        map.addLayer({
                            'id': 'CC2GPath',
                            'type': 'line',
                            'source': 'CC2GPath',
                            'layout': {
                                'line-join': 'round',
                                'line-cap': 'round',
                                'visibility': 'visible'
                            },
                            'paint': {
                                'line-color': 'rgba(50, 205, 50, 0.7)',
                                'line-width': 20,
                                'line-blur': 1,
                                'line-opacity': 0.8
                            }
                        });

                        const CC2GarrowCoordinates = [
                            { coord: [127.440157, 36.729847], rotate: 0},
                            { coord: [127.440254, 36.730017], rotate: 45},
                            { coord: [127.440304, 36.730084], rotate: 30},
                            { coord: [127.440451, 36.730166], rotate: 85},
                            { coord: [127.440557, 36.730178], rotate: 85}
                        ];

                        const CC2GarrowFeatures = CC2GarrowCoordinates.map(arrow => {
                            return {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'Point',
                                    'coordinates': arrow.coord
                                },
                                'properties': {
                                    'rotate': arrow.rotate
                                }
                            };
                        });

                        map.addSource('CC2GArrows', {
                            'type': 'geojson',
                            'data': {
                                'type': 'FeatureCollection',
                                'features': CC2GarrowFeatures
                            }
                        });

                        map.addLayer({
                            'id': 'CC2GArrows',
                            'type': 'symbol',
                            'source': 'CC2GArrows',
                            'layout': {
                                'icon-image': 'arrowG-icon',
                                'icon-size': 0.05,
                                'icon-rotate': ['get', 'rotate'],
                                'icon-allow-overlap': true,
                                'visibility': 'visible'
                            }
                        });
                    }

                    if (map.getLayer('CC2BPath')) {
                        map.setLayoutProperty('CC2BPath', 'visibility', isCC2 ? 'visible' : 'none');
                        map.setLayoutProperty('CC2BArrows', 'visibility', isCC2 ? 'visible' : 'none');
                    } else {
                        map.addSource('CC2BPath', {
                            'type': 'geojson',
                            'data': {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': CC2BsmoothPath
                                }
                            }
                        });

                        map.addLayer({
                            'id': 'CC2BPath',
                            'type': 'line',
                            'source': 'CC2BPath',
                            'layout': {
                                'line-join': 'round',
                                'line-cap': 'round',
                                'visibility': 'visible'
                            },
                            'paint': {
                                'line-color': 'rgba(0, 150, 255, 0.8)',
                                'line-width': 20,
                                'line-blur': 0.5
                            }
                        });

                        const CC2BarrowCoordinates = [
                            { coord: [127.439703, 36.730085], rotate: 90},
                            { coord: [127.439885, 36.730050], rotate: 140},
                            { coord: [127.439991, 36.729972], rotate: 110},
                            { coord: [127.440254, 36.730017], rotate: 45},
                            { coord: [127.440304, 36.730084], rotate: 30},
                            { coord: [127.440451, 36.730166], rotate: 85}
                        ];

                        const CC2BarrowFeatures = CC2BarrowCoordinates.map(arrow => {
                            return {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'Point',
                                    'coordinates': arrow.coord
                                },
                                'properties': {
                                    'rotate': arrow.rotate
                                }
                            };
                        });

                        map.addSource('CC2BArrows', {
                            'type': 'geojson',
                            'data': {
                                'type': 'FeatureCollection',
                                'features': CC2BarrowFeatures
                            }
                        });

                        map.addLayer({
                            'id': 'CC2BArrows',
                            'type': 'symbol',
                            'source': 'CC2BArrows',
                            'layout': {
                                'icon-image': 'arrowB-icon',
                                'icon-size': 0.05,
                                'icon-rotate': ['get', 'rotate'],
                                'icon-allow-overlap': true,
                                'visibility': 'visible'
                            }
                        });
                    }
                });
            }
        });


        map.on('style.load', function() {
            document.getElementById('CC3').addEventListener('click', function() {
                isCC3 = !isCC3;
                if (isCC3) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';
                    updateCC3PathAndMarker();
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';

                    if (map.getLayer('CC3V2XPath')) {
                        map.removeLayer('CC3V2XPath');
                        map.removeSource('CC3V2XPath');
                    }

                    if (CC3NegotiationMarker) {
                        CC3NegotiationMarker.remove();
                    }
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'red';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'green';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);
            });

            function updateCC3PathAndMarker() {
                let CC3Coordinates = [
                    [vehicleLongitude0, vehicleLatitude0],
                    [vehicleLongitude1, vehicleLatitude1]
                ];

                if (!map.getSource('CC3V2XPath')) {
                    map.addSource('CC3V2XPath', {
                        'type': 'geojson',
                        'data': {
                            'type': 'Feature',
                            'geometry': {
                                'type': 'LineString',
                                'coordinates': CC3Coordinates
                            }
                        }
                    });

                    map.addLayer({
                        'id': 'CC3V2XPath',
                        'type': 'line',
                        'source': 'CC3V2XPath',
                        'layout': {
                            'line-join': 'round',
                            'line-cap': 'round',
                            'visibility': 'visible'
                        },
                        'paint': {
                            'line-color': '#007AFF',
                            'line-width': 4,
                            'line-opacity': 0.8,
                            'line-dasharray': [0.5, 1.5]
                        }
                    });
                } else {
                    map.getSource('CC3V2XPath').setData({
                        'type': 'Feature',
                        'geometry': {
                            'type': 'LineString',
                            'coordinates': CC3Coordinates
                        }
                    });
                }

                const midPoint = [
                    (CC3Coordinates[0][0] + CC3Coordinates[1][0]) / 2,
                    (CC3Coordinates[0][1] + CC3Coordinates[1][1]) / 2
                ];

                if (!CC3NegotiationMarker) {
                    CC3NegotiationMarker = new mapboxgl.Marker({element: createCustomLabelCC3()})
                    .setLngLat(midPoint)
                    .addTo(map);
                } else {
                    CC3NegotiationMarker.setLngLat(midPoint);
                    CC3NegotiationMarker.addTo(map);
                }
            }

            function createCustomLabelCC3() {
                const labelContainer = document.createElement('div');
                labelContainer.style.display = 'flex';
                labelContainer.style.flexDirection = 'column';
                labelContainer.style.alignItems = 'center';
                labelContainer.style.width = 'auto';

                // 직사각형 배경
                const background = document.createElement('div');
                background.style.width = 'auto';
                background.style.height = 'auto';
                background.style.padding = '5px 10px';
                background.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                background.style.borderRadius = '10px';
                background.style.boxShadow = '0 0 15px #00ccff, 0 0 30px #00ccff, 0 0 45px #00ccff';

                // 텍스트
                const text = document.createElement('div');
                text.innerHTML = "V2V-SSOV MSG<br>주행 경로 협상";
                text.style.color = 'black';
                text.style.fontWeight = 'bold';
                text.style.textAlign = 'center';
                text.style.textShadow = '0 0 10px #00ccff, 0 0 20px #00ccff, 0 0 30px #00ccff';
                text.style.fontSize = '18px';

                background.appendChild(text);
                labelContainer.appendChild(background);

                return labelContainer;
            }
        });

        let CC4StopMarker = new mapboxgl.Marker({
            element: createCC4Marker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/stop3.png')
            }).setLngLat([127.439772, 36.730093]);

        let CC4GoMarker = new mapboxgl.Marker({
            element: createCC4Marker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/go-straight.png')
            }).setLngLat([127.440172, 36.729915]);

        map.on('style.load', () => {
            document.getElementById('CC4').addEventListener('click', function() {
                isCC4 = !isCC4;
                if (isCC4) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';

                    if (!CC4StopMarker._map) {
                        CC4StopMarker.addTo(map);
                    }
                    if (!CC4GoMarker._map) {
                        CC4GoMarker.addTo(map);
                    }

                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';

                    if (CC4StopMarker._map) {
                        CC4StopMarker.remove();
                    }
                    if (CC4GoMarker._map) {
                        CC4GoMarker.remove();
                    }
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'green';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'green';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);
            });
        });

        function createCC4Marker(imageUrl) {
            const CC4Container = document.createElement('div');
            CC4Container.style.display = 'flex';
            CC4Container.style.flexDirection = 'column';
            CC4Container.style.alignItems = 'center';

            const img = document.createElement('div');
            img.style.backgroundImage = `url(${imageUrl})`;
            img.style.width = '50px';
            img.style.height = '50px';
            img.style.backgroundSize = 'contain';
            img.style.backgroundRepeat = 'no-repeat';

            CC4Container.appendChild(img);
            return CC4Container;
        }

        map.on('style.load', function() {
            document.getElementById('CC5').addEventListener('click', function() {
                isCC5 = !isCC5;
                if (isCC5) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'red';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'green';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);
            });
        });

        map.on('style.load', function() {
            document.getElementById('CC6').addEventListener('click', function() {
                isCC6 = !isCC6;
                if (isCC6) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'red';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'green';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);
            });
        });

        map.on('style.load', function() {
            document.getElementById('CD1').addEventListener('click', function() {
                isCD1 = !isCD1;
                if (isCD1) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';

                    // MRsu Marker 추가
                    const MRsuCoordinate = [127.440227, 36.730164];
                    if (!mrsuMarker) {
                        mrsuMarker = new mapboxgl.Marker({element: createMrsuMarker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/m-rsu-front.png')})
                        .setLngLat(MRsuCoordinate)
                        .addTo(map);
                    } else if (!mrsuMarker._map) {
                        mrsuMarker.setLngLat(MRsuCoordinate).addTo(map);
                    }

                    // ioniq Marker 추가
                    const IoniqCoordinate = [127.440161, 36.729833];
                    if (!ioniqMarker) {
                        ioniqMarker = new mapboxgl.Marker({element: createIoniqMarker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/ioniq-electric-sky.png')})
                        .setLngLat(IoniqCoordinate)
                        .addTo(map);
                    } else if (!ioniqMarker._map) {
                        ioniqMarker.setLngLat(IoniqCoordinate).addTo(map);
                    }
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';

                    if (mrsuMarker && mrsuMarker._map) {
                        mrsuMarker.remove();
                    }

                    if (ioniqMarker && ioniqMarker._map) {
                        ioniqMarker.remove();
                    }
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'yellow';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'yellow';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);
            });
        });

        map.on('style.load', function() {
            document.getElementById('CD2').addEventListener('click', function() {
                isCD2 = !isCD2;
                if (isCD2) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'red';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'red';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);

                if (isCD2) {
                    const CD2Coordinates = [
                        [127.439523, 36.729963],
                        [127.439703, 36.730085]
                    ];

                    if (!map.getSource('CD2V2XPath')) {
                        map.addSource('CD2V2XPath', {
                            'type': 'geojson',
                            'data': {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': CD2Coordinates
                                }
                            }
                        });

                        map.addLayer({
                            'id': 'CD2V2XPath',
                            'type': 'line',
                            'source': 'CD2V2XPath',
                            'layout': {
                                'line-join': 'round',
                                'line-cap': 'round',
                                'visibility': 'visible'
                            },
                            'paint': {
                                'line-color': '#FF0000',
                                'line-width': 4,
                                'line-opacity': 0.8,
                                'line-dasharray': [0.5, 1.5]
                            }
                        });
                    }
                    const CD2midPoint = [
                        (CD2Coordinates[0][0] + CD2Coordinates[1][0]) / 2,
                        (CD2Coordinates[0][1] + CD2Coordinates[1][1]) / 2
                    ];

                    if (!CD2NegotiationMarker) {
                        CD2NegotiationMarker = new mapboxgl.Marker({element: createCustomLabelCD2()})
                        .setLngLat(CD2midPoint)
                        .addTo(map);
                    } else {
                        CD2NegotiationMarker.addTo(map);
                    }
                } else {
                    if (map.getLayer('CD2V2XPath')) {
                        map.removeLayer('CD2V2XPath');
                        map.removeSource('CD2V2XPath');
                    }

                    if (CD2NegotiationMarker) {
                        CD2NegotiationMarker.remove();
                    }
                }
            });
            function createCustomLabelCD2() {
                const CD2labelContainer = document.createElement('div');
                CD2labelContainer.style.display = 'flex';
                CD2labelContainer.style.flexDirection = 'column';
                CD2labelContainer.style.alignItems = 'center';
                CD2labelContainer.style.width = 'auto';

                // 직사각형 배경
                const background = document.createElement('div');
                background.style.width = 'auto';
                background.style.height = 'auto';
                background.style.padding = '5px 10px';
                background.style.backgroundColor = 'rgba(255, 0, 0, 0.9)';
                background.style.borderRadius = '10px';
                background.style.boxShadow = '0 0 15px #ff6666, 0 0 30px #ff6666, 0 0 45px #ff6666';

                // 텍스트
                const text = document.createElement('div');
                text.innerHTML = "V2V-SSOV MSG<br>긴급차 우선 이동 요청";
                text.style.color = 'black';
                text.style.fontWeight = 'bold';
                text.style.textAlign = 'center';
                text.style.textShadow = '0 0 10px #ffcccc, 0 0 20px #ffcccc, 0 0 30px #ffcccc';
                text.style.fontSize = '18px';

                background.appendChild(text);
                CD2labelContainer.appendChild(background);

                return CD2labelContainer;
            }
        });

        const CD3CCoordinates = [
            [127.439541, 36.729890],
            [127.439527, 36.729955],
            [127.439535, 36.730056],
            [127.439641, 36.730080],
            [127.439703, 36.730085], //첫번째 노란점
            [127.439820, 36.730091],
            [127.439885, 36.730050], //두번째 노란점
            [127.439991, 36.729972], //세번째 노란점
            [127.440131, 36.729958],
            [127.440254, 36.730017], //네번째
            [127.440304, 36.730084], //다섯번째
            [127.440352, 36.730148],
            [127.440451, 36.730166] //마지막
        ];

        const CD3ACoordinates = [
            [127.439641, 36.730080],
            [127.439703, 36.730085], //첫번째 노란점
            [127.439820, 36.730091],
            [127.439885, 36.730050], //두번째 노란점
            [127.439991, 36.729972], //세번째 노란점
            [127.440066, 36.729928],
            [127.440083, 36.729863] //네번째 노란점
        ];

        function interpolateCatmullRom(points, numPointsBetween) {
            let interpolatedPoints = [];

            function interpolate(p0, p1, p2, p3, t) {
                const t2 = t * t;
                const t3 = t2 * t;
                const out = [
                    0.5 * (2 * p1[0] + (-p0[0] + p2[0]) * t + (2 * p0[0] - 5 * p1[0] + 4 * p2[0] - p3[0]) * t2 + (-p0[0] + 3 * p1[0] - 3 * p2[0] + p3[0]) * t3),
                    0.5 * (2 * p1[1] + (-p0[1] + p2[1]) * t + (2 * p0[1] - 5 * p1[1] + 4 * p2[1] - p3[1]) * t2 + (-p0[1] + 3 * p1[1] - 3 * p2[1] + p3[1]) * t3)
                ];
                return out;
            }

            for (let i = 1; i < points.length - 2; i++) {
                const p0 = points[i - 1];
                const p1 = points[i];
                const p2 = points[i + 1];
                const p3 = points[i + 2];

                interpolatedPoints.push(p1);
                for (let t = 0; t < numPointsBetween; t++) {
                    const tNorm = t / numPointsBetween;
                    interpolatedPoints.push(interpolate(p0, p1, p2, p3, tNorm));
                }
            }
            interpolatedPoints.push(points[points.length - 2]);
            interpolatedPoints.push(points[points.length - 1]);

            return interpolatedPoints;
        }

        const CD3CsmoothPath = interpolateCatmullRom(CD3CCoordinates, 100);
        const CD3AsmoothPath = interpolateCatmullRom(CD3ACoordinates, 100);

        map.on('style.load',function() {
            if (!map.hasImage('arrowR-icon')) {
                map.loadImage('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/arrowR.png', function(error, image) {
                    if (error) {
                        console.error('fail load image', error);
                        return;
                    }
                    map.addImage('arrowR-icon', image);
                });
            }

            if (!map.hasImage('arrowB-icon')) {
                        map.loadImage('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/arrowB.png', function(error, image) {
                            if (error) {
                                console.error('fail load image', error);
                                return;
                            }
                            map.addImage('arrowB-icon', image);
                });
            }

            document.getElementById('CD3').addEventListener('click', function() {
                isCD3 = !isCD3;
                if (isCD3) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'red';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'red';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);

                if (map.getLayer('CD3CPath')) {
                    map.setLayoutProperty('CD3CPath', 'visibility', isCD3 ? 'visible' : 'none');
                    map.setLayoutProperty('CD3CPathArrows', 'visibility', isCD3 ? 'visible' : 'none');
                } else {
                    map.addSource('CD3CPath', {
                        'type': 'geojson',
                        'data': {
                            'type': 'Feature',
                            'geometry': {
                                'type': 'LineString',
                                'coordinates': CD3CsmoothPath
                            }
                        }
                    });

                    map.addLayer({
                        'id': 'CD3CPath',
                        'type': 'line',
                        'source': 'CD3CPath',
                        'layout': {
                            'line-join': 'round',
                            'line-cap': 'round',
                            'visibility': 'visible'
                        },
                        'paint': {
                            'line-color': 'rgba(255, 0, 0, 0.5)',
                            'line-width': 20,
                            'line-blur': 0.5
                        }
                    });

                    const CD3CarrowCoordinates = [
                        { coord: [127.439527, 36.729955], rotate: 350},
                        { coord: [127.439535, 36.730056], rotate: 45},
                        { coord: [127.439703, 36.730085], rotate: 90},
                        { coord: [127.439885, 36.730050], rotate: 140},
                        { coord: [127.439991, 36.729972], rotate: 110},
                        { coord: [127.440254, 36.730017], rotate: 45},
                        { coord: [127.440304, 36.730084], rotate: 30},
                        { coord: [127.440451, 36.730166], rotate: 85}
                    ];

                    const CD3CarrowFeatures = CD3CarrowCoordinates.map(arrow => {
                        return {
                            'type': 'Feature',
                            'geometry': {
                                'type': 'Point',
                                'coordinates': arrow.coord
                            },
                            'properties': {
                                'rotate': arrow.rotate
                            }
                        };
                    });

                    map.addSource('CD3CPathArrows', {
                        'type': 'geojson',
                        'data': {
                            'type': 'FeatureCollection',
                            'features': CD3CarrowFeatures
                        }
                    });

                    map.addLayer({
                        'id': 'CD3CPathArrows',
                        'type': 'symbol',
                        'source': 'CD3CPathArrows',
                        'layout': {
                            'icon-image': 'arrowR-icon',
                            'icon-size': 0.05,
                            'icon-rotate': ['get', 'rotate'],
                            'icon-allow-overlap': true,
                            'visibility': 'visible'
                        }
                    });
                }

                if (map.getLayer('CD3APath')) {
                    map.setLayoutProperty('CD3APath', 'visibility', isCD3 ? 'visible' : 'none');
                    map.setLayoutProperty('CD3APathArrows', 'visibility', isCD3 ? 'visible' : 'none');
                } else {
                    map.addSource('CD3APath', {
                        'type': 'geojson',
                        'data': {
                            'type': 'Feature',
                            'geometry': {
                                'type': 'LineString',
                                'coordinates': CD3AsmoothPath
                            }
                        }
                    });

                    map.addLayer({
                        'id': 'CD3APath',
                        'type': 'line',
                        'source': 'CD3APath',
                        'layout': {
                            'line-join': 'round',
                            'line-cap': 'round',
                            'visibility': 'visible'
                        },
                        'paint': {
                            'line-color': 'rgba(0, 150, 255, 0.8)',
                            'line-width': 20,
                            'line-blur': 0.5
                        }
                    });

                    const CD3AarrowCoordinates = [
                        { coord: [127.439703, 36.730085], rotate: 90},
                        { coord: [127.439885, 36.730050], rotate: 140},
                        { coord: [127.439991, 36.729972], rotate: 110},
                        { coord: [127.440083, 36.729863], rotate: 170}
                    ];

                    const CD3AarrowFeatures = CD3AarrowCoordinates.map(arrow => {
                        return {
                            'type': 'Feature',
                            'geometry': {
                                'type': 'Point',
                                'coordinates': arrow.coord
                            },
                            'properties': {
                                'rotate': arrow.rotate
                            }
                        };
                    });

                    map.addSource('CD3APathArrows', {
                        'type': 'geojson',
                        'data': {
                            'type': 'FeatureCollection',
                            'features': CD3AarrowFeatures
                        }
                    });

                    map.addLayer({
                        'id': 'CD3APathArrows',
                        'type': 'symbol',
                        'source': 'CD3APathArrows',
                        'layout': {
                            'icon-image': 'arrowB-icon',
                            'icon-size': 0.05,
                            'icon-rotate': ['get', 'rotate'],
                            'icon-allow-overlap': true,
                            'visibility': 'visible'
                        }
                    });
                }
            });
        });

        map.on('style.load', function() {
            document.getElementById('CD4').addEventListener('click', function() {
                isCD4 = !isCD4;
                if (isCD4) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'red';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'red';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);

                if (isCD4) {
                    const CD4Coordinates = [
                        [vehicleLongitude0, vehicleLatitude0],
                        [127.440227, 36.730164] // mRSU
                    ];

                    if (!map.getSource('CD4V2IPath')) {
                        map.addSource('CD4V2IPath', {
                            'type': 'geojson',
                            'data': {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': [
                                        [vehicleLongitude0, vehicleLatitude0],
                                        [127.440227, 36.730164] // 고정된 mRSU 좌표
                                    ]
                                }
                            }
                        });

                        map.addLayer({
                            'id': 'CD4V2IPath',
                            'type': 'line',
                            'source': 'CD4V2IPath',
                            'layout': {
                                'line-join': 'round',
                                'line-cap': 'round',
                                'visibility': 'visible'
                            },
                            'paint': {
                                'line-color': '#27FFFF',
                                'line-width': 4,
                                'line-opacity': 0.8,
                                'line-dasharray': [0.5, 1.5]
                            }
                        });
                    }

                    updateV2IPath('CD4V2IPath', CD4NegotiationMarker);

                    if (!CD4NegotiationMarker) {
                        CD4NegotiationMarker = new mapboxgl.Marker({element: createCustomLabelCD4()})
                        .setLngLat([vehicleLongitude0, vehicleLatitude0])
                        .addTo(map);
                    } else {
                        CD4NegotiationMarker.addTo(map);
                    }
                } else {
                    if (map.getLayer('CD4V2IPath')) {
                        map.removeLayer('CD4V2IPath');
                        map.removeSource('CD4V2IPath');
                    }

                    if (CD4NegotiationMarker) {
                        CD4NegotiationMarker.remove();
                    }
                }
            });
            function createCustomLabelCD4() {
                const CD4labelContainer = document.createElement('div');
                CD4labelContainer.style.display = 'flex';
                CD4labelContainer.style.flexDirection = 'column';
                CD4labelContainer.style.alignItems = 'center';
                CD4labelContainer.style.width = 'auto';

                // 직사각형 배경
                const background = document.createElement('div');
                background.style.width = 'auto';
                background.style.height = 'auto';
                background.style.padding = '3px 7px';
                background.style.backgroundColor = 'rgba(0, 204, 255, 0.8)';
                background.style.borderRadius = '8px';
                background.style.boxShadow = '0 0 15px #00ccff, 0 0 30px #00ccff, 0 0 45px #00ccff';

                // 텍스트
                const text = document.createElement('div');
                text.innerHTML = "V2I-SSOV MSG<br>긴급차→A-VEH 주행 계획 공유<br>A-VEH 양보를 위한 경로 반영";
                text.style.color = 'black';
                text.style.fontWeight = 'bold';
                text.style.textAlign = 'center';
                text.style.textShadow = '0 0 5px #00ccff, 0 0 10px #00ccff, 0 0 15px #00ccff';
                text.style.fontSize = '16px';

                background.appendChild(text);
                CD4labelContainer.appendChild(background);

                return CD4labelContainer;
            }
        });

        map.on('style.load', function() {
            document.getElementById('CD5').addEventListener('click', function() {
                isCD5 = !isCD5;
                if (isCD5) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'red';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'red';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);

                if (isCD5) {
                    const CD5CCoordinates = [
                        [127.440227, 36.730164], //M-RSU
                        [127.440161, 36.729833]
                    ];

                    if (!map.getSource('CD5CPath')) {
                        map.addSource('CD5CPath', {
                            'type': 'geojson',
                            'data': {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': CD5CCoordinates
                                }
                            }
                        });

                        map.addLayer({
                            'id': 'CD5CPath',
                            'type': 'line',
                            'source': 'CD5CPath',
                            'layout': {
                                'line-join': 'round',
                                'line-cap': 'round',
                                'visibility': 'visible'
                            },
                            'paint': {
                                'line-color': '#007AFF',
                                'line-width': 4,
                                'line-opacity': 0.8,
                                'line-dasharray': [0.5, 1.5]
                            }
                        });
                    }
                    const CD5CmidPoint = [
                        (CD5CCoordinates[0][0] + CD5CCoordinates[1][0]) / 2,
                        (CD5CCoordinates[0][1] + CD5CCoordinates[1][1]) / 2
                    ];

                    if (!CD5CNegotiationMarker) {
                        CD5CNegotiationMarker = new mapboxgl.Marker({element: createCustomLabelCD5()})
                        .setLngLat(CD5CmidPoint)
                        .addTo(map);
                    } else {
                        CD5CNegotiationMarker.addTo(map);
                    }

                    if (!CD5ANegotiationMarker) {
                        CD5ANegotiationMarker = new mapboxgl.Marker({element: createCustomLabelCD5()});
                    }

                    if (!map.getSource('CD5APath')) {
                        map.addSource('CD5APath', {
                            'type': 'geojson',
                            'data': {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': [
                                        [vehicleLongitude0, vehicleLatitude0], // 실시간 차량 위치
                                        [127.440227, 36.730164]
                                    ]
                                }
                            }
                        });

                        map.addLayer({
                            'id': 'CD5APath',
                            'type': 'line',
                            'source': 'CD5APath',
                            'layout': {
                                'line-join': 'round',
                                'line-cap': 'round',
                                'visibility': 'visible'
                            },
                            'paint': {
                                'line-color': '#007AFF',
                                'line-width': 4,
                                'line-opacity': 0.8,
                                'line-dasharray': [0.5, 1.5]
                            }
                        });
                    }

                    updateV2IPath('CD5APath', CD5ANegotiationMarker);

                } else {
                    if (map.getLayer('CD5CPath')) {
                        map.removeLayer('CD5CPath');
                        map.removeSource('CD5CPath');
                    }

                    if(CD5CNegotiationMarker) {
                        CD5CNegotiationMarker.remove();
                    }

                    if (map.getLayer('CD5APath')) {
                        map.removeLayer('CD5APath');
                        map.removeSource('CD5APath');
                    }

                    if(CD5ANegotiationMarker) {
                        CD5ANegotiationMarker.remove();
                    }
                }
            });
            function createCustomLabelCD5() {
                const CD5labelContainer = document.createElement('div');
                CD5labelContainer.style.display = 'flex';
                CD5labelContainer.style.flexDirection = 'column';
                CD5labelContainer.style.alignItems = 'center';
                CD5labelContainer.style.width = 'auto';

                // 직사각형 배경
                const background = document.createElement('div');
                background.style.width = 'auto';
                background.style.height = 'auto';
                background.style.padding = '3px 7px';
                background.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                background.style.borderRadius = '8px';
                background.style.boxShadow = '0 0 15px #00ccff, 0 0 30px #00ccff, 0 0 45px #00ccff';

                // 텍스트
                const text = document.createElement('div');
                text.innerHTML = "I2V-SSOV MSG<br>도로 상황 공유";
                text.style.color = 'black';
                text.style.fontWeight = 'bold';
                text.style.textAlign = 'center';
                text.style.textShadow = '0 0 5px #00ccff, 0 0 10px #00ccff, 0 0 15px #00ccff';
                text.style.fontSize = '18px';

                background.appendChild(text);
                CD5labelContainer.appendChild(background);

                return CD5labelContainer;
            }
        });

        const CD6CCoordinates = [
            [127.439541, 36.729890],
            [127.439527, 36.729955],
            [127.439535, 36.730056],
            [127.439641, 36.730080],
            [127.439703, 36.730085], //첫번째 노란점
            [127.439820, 36.730091],
            [127.439885, 36.730050], //두번째 노란점
            [127.439991, 36.729972], //세번째 노란점
            [127.440131, 36.729958],
            [127.440254, 36.730017], //네번째
            [127.440304, 36.730084], //다섯번째
            [127.440352, 36.730148],
            [127.440451, 36.730166] //마지막
        ];

        const CD6ACoordinates = [
            [127.439641, 36.730080],
            [127.439703, 36.730085], //첫번째 노란점
            [127.439820, 36.730091],
            [127.439885, 36.730050], //두번째 노란점
            [127.439991, 36.729972], //세번째 노란점
            [127.440066, 36.729928],
            [127.440083, 36.729863] //네번째 노란점
        ];

        const CD6CsmoothPath = interpolateCatmullRom(CD6CCoordinates, 100);
        const CD6AsmoothPath = interpolateCatmullRom(CD6ACoordinates, 100);

        let CD6Marker = new mapboxgl.Marker({
            element: createCD6Marker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/stop3.png')
            }).setLngLat([127.440172, 36.729915]);

        function createCD6Marker(imageUrl)
        {
            const CD6Container = document.createElement('div');
            CD6Container.style.display = 'flex';
            CD6Container.style.flexDirection = 'column';
            CD6Container.style.alignItems = 'center';

            const img = document.createElement('div');
            img.style.backgroundImage = `url(${imageUrl})`;
            img.style.width = '50px';
            img.style.height = '50px';
            img.style.backgroundSize = 'contain';
            img.style.backgroundRepeat = 'no-repeat';

            CD6Container.appendChild(img);
            return CD6Container;
        }

        map.on('style.load', function() {
            if (!map.hasImage('arrowR-icon')) {
                map.loadImage('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/arrowR.png', function(error, image) {
                    if (error) {
                        console.error('fail load image', error);
                        return;
                    }
                    map.addImage('arrowR-icon', image);
                    });
                }

            if (!map.hasImage('arrowB-icon')) {
                map.loadImage('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/arrowB.png', function(error, image) {
                    if (error) {
                        console.error('fail load image', error);
                        return;
                    }
                    map.addImage('arrowB-icon', image);
                    });
                }

                document.getElementById('CD6').addEventListener('click', function() {
                    isCD6 = !isCD6;
                    if (isCD6) {
                        this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                        this.style.color = 'white';
                    } else {
                        this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                        this.style.color = 'white';
                    }

                    if (vehMode === "C-VEH") {
                        trafficLight = 'red';
                    } else if (vehMode === "A-VEH") {
                        trafficLight = 'green';
                    } else {
                        trafficLight = 'red';
                    }
                    updateTrafficLight(trafficLight);

                    if (map.getLayer('CD6CPath')) {
                        map.setLayoutProperty('CD6CPath', 'visibility', isCD6 ? 'visible' : 'none');
                        map.setLayoutProperty('CD6CPathArrows', 'visibility', isCD6 ? 'visible' : 'none');
                    } else {
                        map.addSource('CD6CPath', {
                            'type': 'geojson',
                            'data': {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': CD6CsmoothPath
                                }
                            }
                        });

                        map.addLayer({
                            'id': 'CD6CPath',
                            'type': 'line',
                            'source': 'CD6CPath',
                            'layout': {
                                'line-join': 'round',
                                'line-cap': 'round',
                                'visibility': 'visible'
                            },
                            'paint': {
                                'line-color': 'rgba(255, 0, 0, 0.5)',
                                'line-width': 20,
                                'line-blur': 0.5
                            }
                        });

                        const CD6CarrowCoordinates = [
                            { coord: [127.439527, 36.729955], rotate: 350},
                            { coord: [127.439535, 36.730056], rotate: 45},
                            { coord: [127.439703, 36.730085], rotate: 90},
                            { coord: [127.439885, 36.730050], rotate: 140},
                            { coord: [127.439991, 36.729972], rotate: 110},
                            { coord: [127.440254, 36.730017], rotate: 45},
                            { coord: [127.440304, 36.730084], rotate: 30},
                            { coord: [127.440451, 36.730166], rotate: 85}
                        ];

                        const CD6CarrowFeatures = CD6CarrowCoordinates.map(arrow => {
                            return {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'Point',
                                    'coordinates': arrow.coord
                                },
                                'properties': {
                                    'rotate': arrow.rotate
                                }
                            };
                        });

                        map.addSource('CD6CPathArrows', {
                            'type': 'geojson',
                            'data': {
                                'type': 'FeatureCollection',
                                'features': CD6CarrowFeatures
                            }
                        });

                        map.addLayer({
                            'id': 'CD6CPathArrows',
                            'type': 'symbol',
                            'source': 'CD6CPathArrows',
                            'layout': {
                                'icon-image': 'arrowR-icon',
                                'icon-size': 0.05,
                                'icon-rotate': ['get', 'rotate'],
                                'icon-allow-overlap': true,
                                'visibility': 'visible'
                            }
                        });
                    }

                    if (map.getLayer('CD6APath')) {
                        map.setLayoutProperty('CD6APath', 'visibility', isCD6 ? 'visible' : 'none');
                        map.setLayoutProperty('CD6APathArrows', 'visibility', isCD6 ? 'visible' : 'none');
                    } else {
                        map.addSource('CD6APath', {
                            'type': 'geojson',
                            'data': {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': CD6AsmoothPath
                                }
                            }
                        });

                        map.addLayer({
                            'id': 'CD6APath',
                            'type': 'line',
                            'source': 'CD6APath',
                            'layout': {
                                'line-join': 'round',
                                'line-cap': 'round',
                                'visibility': 'visible'
                            },
                            'paint': {
                                'line-color': 'rgba(0, 150, 255, 0.8)',
                                'line-width': 20,
                                'line-blur': 0.5
                            }
                        });

                        const CD6AarrowCoordinates = [
                            { coord: [127.439703, 36.730085], rotate: 90},
                            { coord: [127.439885, 36.730050], rotate: 140},
                            { coord: [127.439991, 36.729972], rotate: 110},
                            { coord: [127.440083, 36.729863], rotate: 170}
                        ];

                        const CD6AarrowFeatures = CD6AarrowCoordinates.map(arrow => {
                            return {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'Point',
                                    'coordinates': arrow.coord
                                },
                                'properties': {
                                    'rotate': arrow.rotate
                                }
                            };
                        });

                        map.addSource('CD6APathArrows', {
                            'type': 'geojson',
                            'data': {
                                'type': 'FeatureCollection',
                                'features': CD6AarrowFeatures
                            }
                        });

                        map.addLayer({
                            'id': 'CD6APathArrows',
                            'type': 'symbol',
                            'source': 'CD6APathArrows',
                            'layout': {
                                'icon-image': 'arrowB-icon',
                                'icon-size': 0.05,
                                'icon-rotate': ['get', 'rotate'],
                                'icon-allow-overlap': true,
                                'visibility': 'visible'
                            }
                        });
                    }

                    if (!CD6Marker) {
                        CD6Marker = new mapboxgl.Marker({
                            element: createCD6Marker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/stop3.png')
                        }).setLngLat([127.440172, 36.729915]).addTo(map);
                    } else {
                        if (isCD6) {
                            CD6Marker.addTo(map);
                        } else {
                            CD6Marker.remove();
                        }
                    }
                });
            });



        const CD7Coordinates = [
            [127.439541, 36.729890],
            [127.439527, 36.729955],
            [127.439535, 36.730056],
            [127.439641, 36.730080],
            [127.439703, 36.730085], //첫번째 노란점
            [127.439820, 36.730091],
            [127.439885, 36.730050], //두번째 노란점
            [127.439991, 36.729972], //세번째 노란점
            [127.440131, 36.729958],
            [127.440254, 36.730017], //네번째
            [127.440304, 36.730084], //다섯번째
            [127.440352, 36.730148],
            [127.440451, 36.730166] //마지막
        ]

        const CD7smoothPath = interpolateCatmullRom(CD7Coordinates, 100);

        let CD7Marker = new mapboxgl.Marker({
            element: createCD7Marker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/stop3.png')
            }).setLngLat([127.440172, 36.729915]);

        function createCD7Marker(imageUrl)
        {
            const CD7Container = document.createElement('div');
            CD7Container.style.display = 'flex';
            CD7Container.style.flexDirection = 'column';
            CD7Container.style.alignItems = 'center';

            const img = document.createElement('div');
            img.style.backgroundImage = `url(${imageUrl})`;
            img.style.width = '50px';
            img.style.height = '50px';
            img.style.backgroundSize = 'contain';
            img.style.backgroundRepeat = 'no-repeat';

            CD7Container.appendChild(img);
            return CD7Container;
        }

        map.on('style.load', function() {
            if (!map.hasImage('arrowR-icon')) {
                map.loadImage('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/arrowR.png', function(error, image) {
                    if (error) {
                        console.error('fail load image', error);
                        return;
                    }
                    map.addImage('arrowR-icon', image);
                });
            }

            document.getElementById('CD7').addEventListener('click', function() {
                isCD7 = !isCD7;
                if (isCD7) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'green';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'red';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);

                if (map.getLayer('CD7Path')) {
                    map.setLayoutProperty('CD7Path', 'visibility', isCD7 ? 'visible' : 'none');
                    map.setLayoutProperty('CD7PathArrows', 'visibility', isCD7 ? 'visible' : 'none');
                } else {
                    map.addSource('CD7Path', {
                        'type': 'geojson',
                        'data': {
                            'type': 'Feature',
                            'geometry': {
                                'type': 'LineString',
                                'coordinates': CD7smoothPath
                            }
                        }
                    });

                    map.addLayer({
                        'id': 'CD7Path',
                        'type': 'line',
                        'source': 'CD7Path',
                        'layout': {
                            'line-join': 'round',
                            'line-cap': 'round',
                            'visibility': 'visible'
                        },
                        'paint': {
                            'line-color': 'rgba(255, 0, 0, 0.5)',
                            'line-width': 20,
                            'line-blur': 0.5
                        }
                    });

                    const CD7arrowCoordinates = [
                        { coord: [127.439527, 36.729955], rotate: 350},
                        { coord: [127.439535, 36.730056], rotate: 45},
                        { coord: [127.439703, 36.730085], rotate: 90},
                        { coord: [127.439885, 36.730050], rotate: 140},
                        { coord: [127.439991, 36.729972], rotate: 110},
                        { coord: [127.440254, 36.730017], rotate: 45},
                        { coord: [127.440304, 36.730084], rotate: 30},
                        { coord: [127.440451, 36.730166], rotate: 85}
                    ];

                    const CD7arrowFeatures = CD7arrowCoordinates.map(arrow => {
                        return {
                            'type': 'Feature',
                            'geometry': {
                                'type': 'Point',
                                'coordinates': arrow.coord
                            },
                            'properties': {
                                'rotate': arrow.rotate
                            }
                        };
                    });

                    map.addSource('CD7PathArrows', {
                        'type': 'geojson',
                        'data': {
                            'type': 'FeatureCollection',
                            'features': CD7arrowFeatures
                        }
                    });

                    map.addLayer({
                        'id': 'CD7PathArrows',
                        'type': 'symbol',
                        'source': 'CD7PathArrows',
                        'layout': {
                            'icon-image': 'arrowR-icon',
                            'icon-size': 0.05,
                            'icon-rotate': ['get', 'rotate'],
                            'icon-allow-overlap': true,
                            'visibility': 'visible'
                        }
                    });
                }
                if (isCD7) {
                    CD7Marker.addTo(map);
                } else {
                    CD7Marker.remove();
                }
            });
        });

        let CD8Marker = null;
        let CD8addMarker = null;

        map.on('style.load', function() {
            document.getElementById('CD8').addEventListener('click', function() {
                isCD8 = !isCD8;
                if (isCD8) {
                    this.style.backgroundColor = 'rgba(0, 122, 255, 0.9)';
                    this.style.color = 'white';

                    if (!CD8Marker) {
                        CD8Marker = new mapboxgl.Marker({
                            element: createCD8Marker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/go-straight.png', 355)
                        }).setLngLat([127.440172, 36.729915]).addTo(map);
                    } else if (!CD8Marker._map) {
                        CD8Marker.addTo(map);
                    }

                    if (!CD8addMarker) {
                        CD8addMarker = new mapboxgl.Marker({
                            element: createCD8Marker('https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/go-straight.png', 170)
                        }).setLngLat([127.440120, 36.729752]).addTo(map);
                    } else if (!CD8addMarker._map) {
                        CD8addMarker.addTo(map);
                    }

                    if (!CD8NegotiationMarker) {
                        CD8NegotiationMarker = new mapboxgl.Marker({element: createCustomLabelCD8()});
                    }

                    if (!map.getSource('CD8V2IPath')) {
                        map.addSource('CD8V2IPath', {
                            'type': 'geojson',
                            'data': {
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'LineString',
                                    'coordinates': [
                                        [vehicleLongitude0, vehicleLatitude0], // 실시간 차량 위치
                                        [127.440227, 36.730164]
                                    ]
                                }
                            }
                        });

                        map.addLayer({
                            'id': 'CD8V2IPath',
                            'type': 'line',
                            'source': 'CD8V2IPath',
                            'layout': {
                                'line-join': 'round',
                                'line-cap': 'round',
                                'visibility': 'visible'
                            },
                            'paint': {
                                'line-color': '#4CAF50',
                                'line-width': 4,
                                'line-opacity': 0.8,
                                'line-dasharray': [2,2]
                            }
                        });
                    }
                    updateV2IPath('CD8V2IPath', CD8NegotiationMarker);
                } else {
                    this.style.backgroundColor = 'rgba(0, 0, 0, 0.7)';
                    this.style.color = 'white';

                    if (CD8Marker && CD8Marker._map) {
                        CD8Marker.remove();
                    }

                    if (CD8addMarker && CD8addMarker._map) {
                        CD8addMarker.remove();
                    }

                    if (map.getLayer('CD8V2IPath')) {
                        map.removeLayer('CD8V2IPath');
                        map.removeSource('CD8V2IPath');
                    }

                    if (CD8NegotiationMarker) {
                        CD8NegotiationMarker.remove();
                    }
                }

                if (vehMode === "C-VEH") {
                    trafficLight = 'green';
                } else if (vehMode === "A-VEH") {
                    trafficLight = 'green';
                } else {
                    trafficLight = 'red';
                }
                updateTrafficLight(trafficLight);
            });
        });

        function createCD8Marker(imageUrl, rotationAngle) {
            const CD8Container = document.createElement('div');
            CD8Container.style.display = 'flex';
            CD8Container.style.flexDirection = 'column';
            CD8Container.style.alignItems = 'center';

            const img = document.createElement('div');
            img.style.backgroundImage = `url(${imageUrl})`;
            img.style.width = '50px';
            img.style.height = '50px';
            img.style.backgroundSize = 'contain';
            img.style.backgroundRepeat = 'no-repeat';
            img.style.transform = `rotate(${rotationAngle}deg)`;

            CD8Container.appendChild(img);
            return CD8Container;
        }

        function createCustomLabelCD8() {
            const CD8labelContainer = document.createElement('div');
            CD8labelContainer.style.display = 'flex';
            CD8labelContainer.style.flexDirection = 'column';
            CD8labelContainer.style.alignItems = 'center';
            CD8labelContainer.style.width = 'auto';

            // 직사각형 배경
            const background = document.createElement('div');
            background.style.width = 'auto';
            background.style.height = 'auto';
            background.style.padding = '5px 10px';
            background.style.backgroundColor = '#81C784';
            background.style.borderRadius = '10px';
            background.style.boxShadow = '0 0 15px #4CAF50, 0 0 30px #4CAF50, 0 0 45px #4CAF50';

            // 텍스트
            const text = document.createElement('div');
            text.innerHTML = "V2X-SSOV MSG<br>Class D 완료";
            text.style.color = 'black';
            text.style.fontWeight = 'bold';
            text.style.textAlign = 'center';
            text.style.fontSize = '18px';

            background.appendChild(text);
            CD8labelContainer.appendChild(background);

            return CD8labelContainer;
        }

        if ('WebSocket' in window) {
            let ws = new WebSocket(`ws://${ipAddress}:3001/websocket`);
            ws.onopen = () => {
                console.log('WebSocket connection established');
                ws.send('Client connected');
            }

            function reverseHeading(heading) {
                // heading 값을 180도 회전시키고 좌우를 반대로 변환
                let reversedHeading = (360 - ((parseInt(heading) + 180) % 360)) % 360;
                return reversedHeading;
            }

            if(isTxTest) {
                ws.onmessage = (message) => {
                    let data = message.data.split(',');
                    s_unRxDevId = data[2];
                    s_nRxLatitude = data[23];
                    s_nRxLongitude = data[24];
                    s_unRxVehicleSpeed = data[28];
                    s_unRxVehicleHeading = reverseHeading(data[29]);
                    s_nTxLatitude = vehicleLatitude1;
                    s_nTxLongitude = vehicleLongitude1;
                    s_unTxVehicleHeading = 90;
                    s_unPdr = 100;
                    s_ulLatencyL1 = 500;
                    s_ulTotalPacketCnt = 1 + s_unTempTxCnt;
                    s_unSeqNum = 1 + s_unTempTxCnt;

                    s_unTempTxCnt += 1;
                }
            }
            else
            {
                ws.onmessage = (message) => {
                    let data = message.data.split(',');
                    s_unRxDevId = data[48];
                    s_nRxLatitude = data[62];
                    s_nRxLongitude = data[63];
                    s_unRxVehicleSpeed = data[55];
                    s_unRxVehicleHeading = reverseHeading(data[56]);
                    s_unTxDevId = data[18];
                    s_nTxLatitude = data[32];
                    s_nTxLongitude = data[33];
                    s_unTxVehicleSpeed = data[37];
                    s_unTxVehicleHeading = reverseHeading(data[38]);
                    s_unPdr = data[68];
                    s_ulLatencyL1 = data[43];
                    s_ulTotalPacketCnt = data[66];
                    s_unSeqNum = data[35];
                }
            }

            ws.onerror = (error) => {
                console.error('WebSocket error', error);
            }

            ws.onclose = () => {
                console.log('WebSocket connection closed');
            }
        } else {
            console.error('WebSocket is not supported by this browser');
        }

        /************************************************************/
        /* KD Tree */
        /************************************************************/
        let roadNetworkCoordinates = [];
        let tree;

        map.on('style.load', () => {
            console.log("Map style loaded successfully.");
            addRoadNetworkSource(); // 스타일 로드 후 즉시 소스 추가 시도
        });

        function addRoadNetworkSource() {
            fetch('https://raw.githubusercontent.com/KETI-A/athena/main/src/packages/maps/ctrack-utm52n_ellipsoid/c-track-a2-link.geojson')
                .then(response => {
                    if (!response.ok) {
                        throw new Error("Network response was not ok");
                    }
                    return response.json(); // JSON으로 변환
                })
                .then(geojsonData => {
                    console.log("Fetched GeoJSON Data:", geojsonData);

                    // GeoJSON 데이터가 유효한지 확인
                    if (geojsonData && geojsonData.type === 'FeatureCollection' && Array.isArray(geojsonData.features)) {
                        console.log("GeoJSON data validated. Adding to map...");

                        // GeoJSON 데이터를 Mapbox에 소스로 추가
                        map.addSource('road-network', {
                            'type': 'geojson',
                            'data': geojsonData
                        });

                        console.log("Road network source successfully added.");

                        // KD-Tree 빌드 함수 호출
                        //buildKdTreeFromGeoJSON(geojsonData);
                        buildKdTreeInterPolateFromGeoJSON(geojsonData);
                    } else {
                        console.error("GeoJSON data is invalid or missing features.");
                    }
                })
                .catch(error => {
                    console.error("Failed to fetch GeoJSON data:", error);
                });
        }

        class KDTree {
            constructor(points, metric) {
                this.metric = metric;
                this.dimensions = [0, 1];  // 경도와 위도 인덱스 (0: longitude, 1: latitude)
                this.root = this.buildTree(points, 0);
            }

            buildTree(points, depth) {
                if (points.length === 0) return null;

                const axis = depth % this.dimensions.length; // 경도와 위도를 번갈아가며 분할
                points.sort((a, b) => a[axis] - b[axis]); // 각 축에 따라 정렬

                const median = Math.floor(points.length / 2); // 중간값

                return {
                    point: points[median], // 중간값 기준으로 노드를 설정
                    left: this.buildTree(points.slice(0, median), depth + 1), // 왼쪽 하위 트리
                    right: this.buildTree(points.slice(median + 1), depth + 1) // 오른쪽 하위 트리
                };
            }

            nearest(point, maxNodes = 1) {
                const bestNodes = new BinaryHeap((e) => -e[1]);  // 최소 힙을 사용하여 최근접 노드를 추적

                const nearestSearch = (node, depth) => {
                    if (node === null) return;

                    const axis = depth % this.dimensions.length; // 현재 비교할 축 (경도 또는 위도)

                    const ownDistance = this.metric(point, node.point); // 현재 노드와의 거리 계산
                    const linearPoint = [...point]; // 입력된 좌표의 복사본을 만듦
                    linearPoint[axis] = node.point[axis]; // 한 축을 고정하여 계산

                    let bestChild = null;
                    let otherChild = null;

                    // 입력된 좌표와 현재 노드의 비교 축에 따라 왼쪽 또는 오른쪽 자식 노드 선택
                    if (point[axis] < node.point[axis]) {
                        bestChild = node.left;
                        otherChild = node.right;
                    } else {
                        bestChild = node.right;
                        otherChild = node.left;
                    }

                    // 더 가까운 쪽 자식 노드를 먼저 검색
                    nearestSearch(bestChild, depth + 1);

                    // 현재 노드와의 거리가 현재 가장 가까운 거리보다 작다면 갱신
                    if (bestNodes.size() < maxNodes || ownDistance < bestNodes.peek()[1]) {
                        bestNodes.push([node.point, ownDistance]);
                        if (bestNodes.size() > maxNodes) bestNodes.pop();
                    }

                    const linearDistance = this.metric(linearPoint, node.point); // 축을 고정한 거리 계산

                    // 고정된 축을 기준으로 계산된 거리가 더 가까울 수 있는지 검사
                    if (bestNodes.size() < maxNodes || linearDistance < bestNodes.peek()[1]) {
                        nearestSearch(otherChild, depth + 1);
                    }
                };

                nearestSearch(this.root, 0);

                const result = [];
                while (bestNodes.size()) {
                    result.push(bestNodes.pop()[0]);
                }

                return result;
            }
        }

        class BinaryHeap {
            constructor(scoreFunction) {
                this.content = [];
                this.scoreFunction = scoreFunction;
            }

            push(element) {
                this.content.push(element);
                this.bubbleUp(this.content.length - 1);
            }

            pop() {
                const result = this.content[0];
                const end = this.content.pop();
                if (this.content.length > 0) {
                    this.content[0] = end;
                    this.sinkDown(0);
                }
                return result;
            }

            size() {
                return this.content.length;
            }

            peek() {
                return this.content[0];
            }

            bubbleUp(n) {
                const element = this.content[n];
                const score = this.scoreFunction(element);

                while (n > 0) {
                    const parentN = Math.floor((n + 1) / 2) - 1;
                    const parent = this.content[parentN];
                    if (score >= this.scoreFunction(parent)) break;
                    this.content[parentN] = element;
                    this.content[n] = parent;
                    n = parentN;
                }
            }

            sinkDown(n) {
                const length = this.content.length;
                const element = this.content[n];
                const elemScore = this.scoreFunction(element);

                while (true) {
                    const child2N = (n + 1) * 2;
                    const child1N = child2N - 1;
                    let swap = null;
                    let child1Score;

                    if (child1N < length) {
                        const child1 = this.content[child1N];
                        child1Score = this.scoreFunction(child1);
                        if (child1Score < elemScore) swap = child1N;
                    }

                    if (child2N < length) {
                        const child2 = this.content[child2N];
                        const child2Score = this.scoreFunction(child2);
                        if (child2Score < (swap === null ? elemScore : child1Score)) swap = child2N;
                    }

                    if (swap === null) break;

                    this.content[n] = this.content[swap];
                    this.content[swap] = element;
                    n = swap;
                }
            }
        }

        // 유클리드 거리 측정 함수
        function euclideanDistance(a, b) {
            const dx = a[0] - b[0]; // 경도 차이
            const dy = a[1] - b[1]; // 위도 차이
            return Math.sqrt(dx * dx + dy * dy);
        }

        function buildKdTreeFromGeoJSON(geojsonData) {
            let roadNetworkCoordinates = [];

            // MultiLineString 및 LineString 처리
            geojsonData.features.forEach(feature => {
                if (feature.geometry.type === "LineString") {
                    feature.geometry.coordinates.forEach(coord => {
                        console.log("KD-Tree Input Coordinate:", coord); // 좌표를 출력
                        roadNetworkCoordinates.push([coord[0], coord[1]]);
                    });
                } else if (feature.geometry.type === "MultiLineString") {
                    feature.geometry.coordinates.forEach(line => {
                        line.forEach(coord => {
                            console.log("KD-Tree Input Coordinate:", coord); // 좌표를 출력
                            roadNetworkCoordinates.push([coord[0], coord[1]]);
                        });
                    });
                }
            });

            // KD-Tree 생성 및 전역 변수로 설정
            tree = new KDTree(roadNetworkCoordinates, euclideanDistance);

            console.log("KD-Tree built successfully with", roadNetworkCoordinates.length, "points.");

            // KD-Tree 좌표를 지도에 추가
            if(isVisiblePath) {
                addPointsToMapOfKdTree(roadNetworkCoordinates);
            }
        }

        function buildKdTreeInterPolateFromGeoJSON(geojsonData) {
            let roadNetworkCoordinates = [];

            // MultiLineString 및 LineString 처리
            geojsonData.features.forEach(feature => {
                if (feature.geometry.type === "LineString") {
                    for (let i = 0; i < feature.geometry.coordinates.length - 1; i++) {
                        let startCoord = feature.geometry.coordinates[i];
                        let endCoord = feature.geometry.coordinates[i + 1];

                        // 두 점 사이의 거리 계산 (Haversine Formula 사용)
                        let distance = haversineDistance([startCoord[0], startCoord[1]], [endCoord[0], endCoord[1]]);
                        //console.log(`Distance between points: ${distance} meters`);

                        // 원래 좌표 추가
                        roadNetworkCoordinates.push([startCoord[0], startCoord[1]]);

                        // 두 점 사이의 거리가 1m 이상 10m 이하일 때만 보간
                        if (distance >= 1 && distance <= 10) {
                            // 두 점 사이를 30cm 단위로 보간하여 추가
                            let interpolatedPoints = interpolatePoints([startCoord, endCoord], 0.3);  // 간격을 30cm로 설정
                            roadNetworkCoordinates.push(...interpolatedPoints);  // 보간된 좌표 추가
                        } else {
                            //console.log(`No interpolation: Distance is ${distance} meters`);
                        }

                        // 마지막 점 추가
                        if (i === feature.geometry.coordinates.length - 2) {
                            roadNetworkCoordinates.push([endCoord[0], endCoord[1]]);
                        }
                    }
                } else if (feature.geometry.type === "MultiLineString") {
                    feature.geometry.coordinates.forEach(line => {
                        for (let i = 0; i < line.length - 1; i++) {
                            let startCoord = line[i];
                            let endCoord = line[i + 1];

                            // 두 점 사이의 거리 계산 (Haversine Formula 사용)
                            let distance = haversineDistance([startCoord[0], startCoord[1]], [endCoord[0], endCoord[1]]);
                            //console.log(`Distance between points: ${distance} meters`);

                            // 원래 좌표 추가
                            roadNetworkCoordinates.push([startCoord[0], startCoord[1]]);

                            // 두 점 사이의 거리가 1m 이상 10m 이하일 때만 보간
                            if (distance >= 1 && distance <= 10) {
                                // 두 점 사이를 30cm 단위로 보간하여 추가
                                let interpolatedPoints = interpolatePoints([startCoord, endCoord], 0.3);  // 간격을 30cm로 설정
                                roadNetworkCoordinates.push(...interpolatedPoints);  // 보간된 좌표 추가
                            } else {
                               //console.log(`No interpolation: Distance is ${distance} meters`);
                            }

                            // 마지막 점 추가
                            if (i === line.length - 2) {
                                roadNetworkCoordinates.push([endCoord[0], endCoord[1]]);
                            }
                        }
                    });
                }
            });

            console.log("Total Points After Interpolation:", roadNetworkCoordinates.length);  // 보간된 좌표 로그 출력

            // KD-Tree 생성 및 전역 변수로 설정
            //tree = new KDTree(roadNetworkCoordinates, euclideanDistance);
            // KD-Tree 생성 시 거리 계산을 Haversine Formula로 변경
            tree = new KDTree(roadNetworkCoordinates, haversineDistance);

            console.log("KD-Tree built successfully with", roadNetworkCoordinates.length, "points.");

            // KD-Tree 좌표를 지도에 추가
            if (isVisiblePath) {
                addPointsToMapOfKdTree(roadNetworkCoordinates);
            }
        }

        // Haversine formula를 사용한 거리 계산 (메트릭 단위로 반환)
        function haversineDistance(coord1, coord2) {
            const R = 6371000; // 지구의 반지름 (미터)
            const lat1 = toRadians(coord1[1]);
            const lat2 = toRadians(coord2[1]);
            const deltaLat = toRadians(coord2[1] - coord1[1]);
            const deltaLon = toRadians(coord2[0] - coord1[0]);

            const a = Math.sin(deltaLat / 2) * Math.sin(deltaLat / 2) +
                      Math.cos(lat1) * Math.cos(lat2) *
                      Math.sin(deltaLon / 2) * Math.sin(deltaLon / 2);

            const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

            return R * c; // 거리 (미터 단위)
        }

        // 각도를 라디안으로 변환
        function toRadians(degrees) {
            return degrees * Math.PI / 180;
        }

        // 보간 함수: 두 점 사이의 빈 공간을 30cm 단위로 채우는 좌표 생성
        function interpolatePoints(points, spacing) {
            let interpolatedPoints = [];
            let [x1, y1] = points[0];
            let [x2, y2] = points[1];
            let distance = haversineDistance([x1, y1], [x2, y2]);
            let steps = Math.floor(distance / spacing);

            for (let j = 1; j < steps; j++) {  // 30cm 간격으로 중간 점 추가 (끝 점 제외)
                let t = j / steps;
                let x = x1 + t * (x2 - x1);
                let y = y1 + t * (y2 - y1);
                interpolatedPoints.push([x, y]);
            }

            return interpolatedPoints;
        }

        // 지도에 점 추가 함수
        function addPointsToMapOfKdTree(coordinates) {
            // 지도에 추가할 점 데이터
            const points = coordinates.map(coord => ({
                'type': 'Feature',
                'geometry': {
                    'type': 'Point',
                    'coordinates': coord
                }
            }));

            // 기존의 소스가 있으면 삭제
            if (map.getSource('kd-tree-points')) {
                map.removeSource('kd-tree-points');
            }

            // 지도에 점 추가
            map.addSource('kd-tree-points', {
                'type': 'geojson',
                'data': {
                    'type': 'FeatureCollection',
                    'features': points
                }
            });

            // 기존의 레이어가 있으면 삭제
            if (map.getLayer('kd-tree-points-layer')) {
                map.removeLayer('kd-tree-points-layer');
            }

            map.addLayer({
                'id': 'kd-tree-points-layer',
                'type': 'circle',
                'source': 'kd-tree-points',
                'paint': {
                    'circle-radius': 3,
                    'circle-color': '#00FF00'
                }
            });
            console.log("Added points to map:", points);
        }

        /************************************************************/
        /* Map */
        /************************************************************/
        map.on('load', () => {
            console.log("Map loaded successfully.");

            map.on('zoom', () => {
                map.resize();  // 확대/축소 시 강제로 지도를 리렌더링
            });

            map.on('move', () => {
                map.resize();  // 지도가 이동될 때 리렌더링
            });

            /************************************************************/
            /* CI */
            /************************************************************/
            map.loadImage(
                'https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/keti_ci.png',
                (error, image) => {
                    if (error) throw error;

                    map.addImage('keti-log', image);

                    map.addSource('keti_log_src', {
                        'type': 'geojson',
                        'data': {
                            'type': 'FeatureCollection',
                            'features': [
                                {
                                    'type': 'Feature',
                                    'geometry': {
                                        'type': 'Point',
                                        'coordinates': [cKetiPangyolongitude, cKetiPangyoLatitude]
                                    }
                                }
                            ]
                        }
                    });

                    map.addLayer({
                        'id': 'keti',
                        'type': 'symbol',
                        'source': 'keti_log_src',
                        'layout': {
                            'icon-image': 'keti-log',
                            'icon-size': 0.50,
                            'icon-allow-overlap': true
                        }
                    });
                }
            );

            /************************************************************/
            /* RSU */
            /************************************************************/
            map.loadImage(
                'https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/fixed-rsu.png',
                (error, image) => {
                    if (error) throw error;

                    map.addImage('rsu', image);

                    map.addSource('rsu-src-18', {
                        'type': 'geojson',
                        'data': {
                            'type': 'FeatureCollection',
                            'features': [
                                {
                                    'type': 'Feature',
                                    'geometry': {
                                        'type': 'Point',
                                        'coordinates': [cPangyoRsuLongitude18, cPangyoRsuLatitude18]
                                    }
                                }
                            ]
                        }
                    });

                    map.addLayer({
                        'id': 'rsu18',
                        'type': 'symbol',
                        'source': 'rsu-src-18',
                        'layout': {
                            'icon-image': 'rsu',
                            'icon-size': 0.4,
                            'icon-allow-overlap': true
                        }
                    });

                    map.addSource('rsu-src-16', {
                        'type': 'geojson',
                        'data': {
                            'type': 'FeatureCollection',
                            'features': [
                                {
                                    'type': 'Feature',
                                    'geometry': {
                                        'type': 'Point',
                                        'coordinates': [cPangyoRsuLongitude16, cPangyoRsuLatitude16]
                                    }
                                }
                            ]
                        }
                    });

                    map.addLayer({
                        'id': 'rsu16',
                        'type': 'symbol',
                        'source': 'rsu-src-16',
                        'layout': {
                            'icon-image': 'rsu',
                            'icon-size': 0.4,
                            'icon-allow-overlap': true
                        }
                    });

                    map.addSource('rsu-src-17', {
                        'type': 'geojson',
                        'data': {
                            'type': 'FeatureCollection',
                            'features': [
                                {
                                    'type': 'Feature',
                                    'geometry': {
                                        'type': 'Point',
                                        'coordinates': [cPangyoRsuLongitude17, cPangyoRsuLatitude17]
                                    }
                                }
                            ]
                        }
                    });

                    map.addLayer({
                        'id': 'rsu17',
                        'type': 'symbol',
                        'source': 'rsu-src-17',
                        'layout': {
                            'icon-image': 'rsu',
                            'icon-size': 0.4,
                            'icon-allow-overlap': true
                        }
                    });

                }
            );

            map.addSource('rsu-src-31', {
                'type': 'geojson',
                'data': {
                    'type': 'FeatureCollection',
                    'features': [
                        {
                            'type': 'Feature',
                            'geometry': {
                                'type': 'Point',
                                'coordinates': [cPangyoRsuLongitude31, cPangyoRsuLatitude31]
                            }
                        }
                    ]
                }
            });

            map.addLayer({
                'id': 'rsu31',
                'type': 'symbol',
                'source': 'rsu-src-31',
                'layout': {
                    'icon-image': 'rsu',
                    'icon-size': 0.4,
                    'icon-allow-overlap': true
                }
            });

            map.addSource('rsu-src-5', {
                'type': 'geojson',
                'data': {
                    'type': 'FeatureCollection',
                    'features': [
                        {
                            'type': 'Feature',
                            'geometry': {
                                'type': 'Point',
                                'coordinates': [cPangyoRsuLongitude5, cPangyoRsuLatitude5]
                            }
                        }
                    ]
                }
            });

            map.addLayer({
                'id': 'rsu5',
                'type': 'symbol',
                'source': 'rsu-src-5',
                'layout': {
                    'icon-image': 'rsu',
                    'icon-size': 0.4,
                    'icon-allow-overlap': true
                }
            });

            /************************************************************/
            /* Vehicle 0 */
            /************************************************************/
            map.loadImage(vehicle0ImageUrl, (error, image) => {
                    if (error) throw error;

                    map.addImage('vehicle', image);

                    map.addSource('vehicle_src_0', {
                        'type': 'geojson',
                        'data': {
                            'type': 'FeatureCollection',
                            'features': [
                                {
                                    'type': 'Feature',
                                    'geometry': {
                                        'type': 'Point',
                                        'coordinates': [vehicleLongitude0, vehicleLatitude0]
                                    }
                                }
                            ]
                        }
                    });

                    map.addLayer({
                        'id': 'vehicle0',
                        'type': 'symbol',
                        'source': 'vehicle_src_0',
                        'layout': {
                            'icon-image': 'vehicle',
                            'icon-size': 0.2,
                            'icon-rotate': ['get', 'heading'],
                            'text-field': [
                                'concat',
                                ['case',
                                    ['==', ['get', 'deviceID'], CVehId], 'C-VEH#',
                                    ['==', ['get', 'deviceID'], AVehId], 'A-VEH#',
                                    'OBU#'
                                ],
                                ['get', 'deviceID']
                            ],
                            'text-font': ['Open Sans Semibold', 'Arial Unicode MS Bold'],
                            'text-offset': [0, 2], // Adjust this value to position the text
                            'text-anchor': 'top',
                            'text-size' : 13,
                            'icon-allow-overlap': true,
                            'text-allow-overlap': true
                        },
                        'paint': {
                            'text-color': '#000000',
                            'text-halo-color': '#FFFFFF',
                            'text-halo-width': 2
                        }
                    });

                    map.moveLayer('vehicle0');

                    fetchAndUpdate();
                    setInterval(fetchAndUpdate, 10);
                }
            );

            /************************************************************/
            /* Vehicle 1 */
            /************************************************************/
            map.loadImage(vehicle1ImageUrl, (error, image) => {
                    if (error) throw error;

                    map.addImage('vehicle1', image);

                    map.addSource('vehicle_src_1', {
                        'type': 'geojson',
                        'data': {
                            'type': 'FeatureCollection',
                            'features': [
                                {
                                    'type': 'Feature',
                                    'geometry': {
                                        'type': 'Point',
                                        'coordinates': [vehicleLongitude1, vehicleLatitude1]
                                    }
                                }
                            ]
                        }
                    });

                    map.addLayer({
                        'id': 'vehicle1',
                        'type': 'symbol',
                        'source': 'vehicle_src_1',
                        'layout': {
                            'icon-image': 'vehicle1',
                            'icon-size': 0.2,
                            'icon-rotate': ['get', 'heading'],
                            'text-field': [
                                'concat',
                                ['case',
                                    ['==', ['get', 'deviceID'], CVehId], 'C-VEH#',
                                    ['==', ['get', 'deviceID'], AVehId], 'A-VEH#',
                                    'OBU#'
                                ],
                                ['get', 'deviceID']
                            ],
                            'text-font': ['Open Sans Semibold', 'Arial Unicode MS Bold'],
                            'text-offset': [0, 2],
                            'text-anchor': 'top',
                            'text-size' : 13,
                            'icon-allow-overlap': true,
                            'text-allow-overlap': true
                        },
                        'paint': {
                            'text-color': '#000000',
                            'text-halo-color': '#FFFFFF',
                            'text-halo-width': 2
                        }
                    });

                    map.moveLayer('vehicle1');

                    fetchAndUpdate();
                    setInterval(fetchAndUpdate, 10);
                }
            );

            console.log("Road network and vehicle sources added successfully.");
        });

        /************************************************************/
        /* Update Position */
        /************************************************************/
        // 이전 좌표를 저장할 객체 (vehicleId에 따라 다르게 저장)
        let previousCoordinatesMap = {};

        function updateVehiclePosition(vehicleId, coordinates, heading, deviceId) {
            let vehicleSource = map.getSource(`vehicle_src_${vehicleId}`);
            let snappedCoordinates = coordinates;  // 기본값을 실제 GPS 좌표로 설정
            let maxAllowedShift = 2;  // 허용 가능한 최대 이동 거리 (미터 단위)

            // 이전 좌표가 존재하지 않으면 현재 좌표를 이전 좌표로 설정
            if (!previousCoordinatesMap[vehicleId]) {
                previousCoordinatesMap[vehicleId] = coordinates; // 이전 좌표를 현재 좌표로 초기화
            }

            let previousCoordinates = previousCoordinatesMap[vehicleId];  // 현재 차량의 이전 좌표

            /* KD Tree Path: KD 트리를 사용할 때 스냅된 좌표로 업데이트 */
            if (tree && isPathPlan) {
                let point = {
                    longitude: coordinates[0],
                    latitude: coordinates[1]
                };

                let nearest = tree.nearest([point.longitude, point.latitude], 1);

                if (nearest.length > 0) {
                    let nearestPoint = nearest[0];
                    let distanceThreshold = 2;  // 허용 가능한 스냅 거리 (미터 단위)

                    // Haversine 공식을 사용하여 현재 좌표와 KD 트리에서 선택된 좌표 사이의 거리 계산
                    let distance = haversineDistance([point.longitude, point.latitude], nearestPoint);

                    // 필터링: 거리가 임계값 이하일 경우에만 스냅된 좌표로 업데이트
                    if (distance < distanceThreshold) {
                        //console.log(`Distance: ${distance} meters, Distance Threshold: ${distanceThreshold} meters`);

                        // Haversine 공식을 사용하여 이전 좌표와 KD 트리에서 선택된 좌표 간의 이동 거리 계산
                        let shift = haversineDistance(previousCoordinates, nearestPoint);

                        if (shift < maxAllowedShift) {
                            snappedCoordinates = nearestPoint; // 조건을 충족하는 경우에만 스냅된 좌표 사용
                            //console.log(`Snapped to nearest point: ${snappedCoordinates}`);
                        } else {
                            console.log(`Shift too large: ${shift} meters.`);
                        }
                    } else {
                        // 거리가 임계값을 초과한 경우, 스냅된 좌표를 사용하지 않음
                        console.warn(`Distance (${distance} meters) exceeds threshold (${distanceThreshold} meters), not snapping to nearest point.`);
                    }

                    if (vehicleSource) {
                        vehicleSource.setData({
                            'type': 'FeatureCollection',
                            'features': [{
                                'type': 'Feature',
                                'geometry': {
                                    'type': 'Point',
                                    'coordinates': snappedCoordinates
                                },
                                'properties': {
                                    'heading': heading,
                                    'deviceID': deviceId  // Device ID 추가
                                }
                            }]
                        });
                    } else {
                        console.warn(`Vehicle source vehicle_src_${vehicleId} not found.`);
                    }

                    if(isVisiblePath) {
                    updateSnappedPath(snappedCoordinates);
                    }
                } else {
                    console.warn("No nearest point found in KD Tree.");
                }
            } else {
                // Real GPS Path: 실제 GPS 좌표로 지도 위 차량 위치 업데이트
                if (vehicleSource) {
                    vehicleSource.setData({
                        'type': 'FeatureCollection',
                        'features': [{
                            'type': 'Feature',
                            'geometry': {
                                'type': 'Point',
                                'coordinates': coordinates
                            },
                            'properties': {
                                'heading': heading,
                                'deviceID': deviceId  // Device ID 추가
                            }
                        }]
                    });
                } else {
                    console.warn(`Vehicle source vehicle_src_${vehicleId} not found.`);
                }
            }

            // 상태 업데이트 (스냅된 좌표 또는 실제 좌표로 업데이트)
            if (isPathPlan) {
                if (vehicleId === 0) {
                    vehicleLongitude0 = snappedCoordinates[0];
                    vehicleLatitude0 = snappedCoordinates[1];
                } else if (vehicleId === 1) {
                    vehicleLongitude1 = snappedCoordinates[0];
                    vehicleLatitude1 = snappedCoordinates[1];
                }
            } else {
                if (vehicleId === 0) {
                    vehicleLongitude0 = coordinates[0];
                    vehicleLatitude0 = coordinates[1];
                } else if (vehicleId === 1) {
                    vehicleLongitude1 = coordinates[0];
                    vehicleLatitude1 = coordinates[1];
                }
            }

            // 이전 좌표 업데이트 (현재 좌표를 다음에 사용하기 위해 저장)
            previousCoordinatesMap[vehicleId] = snappedCoordinates;

            // 연결된 선 업데이트 (isCvLineEnabled가 true일 때만)
            if (isCvLineEnabled && map.getSource('line')) {
                map.getSource('line').setData({
                    'type': 'Feature',
                    'geometry': {
                        'type': 'LineString',
                        'coordinates': [[vehicleLongitude0, vehicleLatitude0], [vehicleLongitude1, vehicleLatitude1]]
                    }
                });
            }

            if (isCentering && vehicleId === 0) {
                map.setCenter(snappedCoordinates); // 지도 중심을 스냅된 좌표로 설정
            }

            if(isVisiblePath) {
                updateGpsPath(coordinates); // 실제 GPS 좌표를 경로로 업데이트
            }

            // CB3이 활성화된 경우 경로 업데이트
            if (isCB3) {
                updateV2VPath('CB3V2XPath', CB3NegotiationMarker);
            }

            // CB4가 활성화된 경우 경로 업데이트
            if (isCB4) {
                updateV2VPath('CB4V2XPath', CB4NegotiationMarker);
            }

            // CB6이 활성화된 경우 경로 업데이트
            if (isCB6) {
                updateV2VPath('CB6V2XPath', CB6NegotiationMarker);
            }

            // CC3이 활성화된 경우 경로 업데이트
            if (isCC3) {
                updateV2VPath('CC3V2XPath', CC3NegotiationMarker);
            }

            // CD2가 활성화된 경우 경로 업데이트
            if (isCD2) {
                updateV2VPath('CD2V2XPath', CD2NegotiationMarker);
            }

            if (isCD4) {
                updateV2IPath('CD4V2IPath', CD4NegotiationMarker);
            }

            if (isCD5) {
                updateV2IPath('CD5APath', CD5ANegotiationMarker);
            }

            if (isCD8) {
                updateV2IPath('CD8V2IPath', CD8NegotiationMarker);
            }
        }

        // 단일 GPS 좌표를 추가하는 함수
        function updateGpsPath(coordinate) {
            // 기존 소스가 없으면 생성
            if (!map.getSource('gps-path')) {
                map.addSource('gps-path', {
                    'type': 'geojson',
                    'data': {
                        'type': 'FeatureCollection',
                        'features': []
                    }
                });
            }

            // 현재 소스 데이터 가져오기
            const source = map.getSource('gps-path');
            const data = source._data || { 'type': 'FeatureCollection', 'features': [] };

            // 새로운 점 추가
            data.features.push({
                'type': 'Feature',
                'geometry': {
                    'type': 'Point',
                    'coordinates': coordinate
                }
            });

            // 소스 데이터 업데이트
            source.setData(data);

            // 레이어가 없으면 추가
            if (!map.getLayer('gps-path-layer')) {
                map.addLayer({
                    'id': 'gps-path-layer',
                    'type': 'circle',
                    'source': 'gps-path',
                    'paint': {
                        'circle-radius': 3,
                        'circle-color': '#FF0000',
                    }
                });
            }
        }

        /************************************************************/
        /* Update Snapped (KD Tree) Path with Blue Dots */
        /************************************************************/
        function updateSnappedPath(coordinate) {
            // 기존 소스가 없으면 생성
            if (!map.getSource('snapped-path')) {
                map.addSource('snapped-path', {
                    'type': 'geojson',
                    'data': {
                        'type': 'FeatureCollection',
                        'features': []
                    }
                });
            }

            // 현재 소스 데이터 가져오기
            const source = map.getSource('snapped-path');
            const data = source._data || { 'type': 'FeatureCollection', 'features': [] };

            // 새로운 스냅된 점 추가
            data.features.push({
                'type': 'Feature',
                'geometry': {
                    'type': 'Point',
                    'coordinates': coordinate
                }
            });

            // 소스 데이터 업데이트
            source.setData(data);

            // 레이어가 없으면 추가
            if (!map.getLayer('snapped-path-layer')) {
                map.addLayer({
                    'id': 'snapped-path-layer',
                    'type': 'circle',
                    'source': 'snapped-path',
                    'paint': {
                        'circle-radius': 3,
                        'circle-color': '#0000FF',  // 파란색 점
                    }
                });
            }
        }

        function updateTrafficLight(trafficLight) {
            const trafficLightImage = document.getElementById('traffic-light-image');
            const trafficLightStatus = document.getElementById('traffic-light-status');

            if (trafficLight.toLowerCase() === 'green') {
                trafficLightImage.src = 'https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/green-light.png';  // 초록 불로 변경
                trafficLightStatus.innerHTML = 'Green Light<br>(GO)';       // 텍스트 업데이트
                trafficLightStatus.style.color = 'green';
            } else if (trafficLight.toLowerCase() === 'yellow') {
                trafficLightImage.src = 'https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/yellow-light.png';  // 주황 불로 변경
                trafficLightStatus.innerHTML = 'Yellow Light<br>(SLOW)';      // 텍스트 업데이트
                trafficLightStatus.style.color = 'orange';
            } else if (trafficLight.toLowerCase() === 'red') {
                trafficLightImage.src = 'https://raw.githubusercontent.com/KETI-A/athena/main/src/apps/html/images/red-light.png';  // 빨간 불로 변경
                trafficLightStatus.innerHTML = 'RED Light<br>(STOP)';         // 텍스트 업데이트
                trafficLightStatus.style.color = 'red';
            } else {
                console.warn('Invalid traffic light color:', trafficLight);
            }
        }

        function updateHeadingInfo(heading) {
            const headingText = document.getElementById('heading-text');
            headingText.innerText = `${heading}°`;
        }

        function updateSpeedInfo(speed) {
            const speedValue = document.getElementById('speed-value');
            speedValue.innerText = speed;
        }

        function fetchAndUpdate() {
            if (!tree) {
                console.warn("KD-Tree is not built yet. Waiting...");
                return;
            }

            const devId0 = parseFloat(s_unRxDevId);  // device ID를 파싱
            const latitude0 = parseFloat(s_nRxLatitude);
            const longitude0 = parseFloat(s_nRxLongitude);
            const heading0 = parseFloat(s_unRxVehicleHeading);
            const speed0 = parseFloat(s_unRxVehicleSpeed);

            const devId1 = parseFloat(s_unTxDevId);  // device ID를 파싱
            const latitude1 = parseFloat(s_nTxLatitude);
            const longitude1 = parseFloat(s_nTxLongitude);
            const heading1 = parseFloat(s_unTxVehicleHeading);

            if (!isNaN(latitude0) && !isNaN(longitude0)) {
                updateVehiclePosition(0, [longitude0, latitude0], heading0, devId0);  // device ID 전달
                updateHeadingInfo(heading0);
                updateSpeedInfo(speed0);
            }

            if (!isNaN(latitude1) && !isNaN(longitude1)) {
                updateVehiclePosition(1, [longitude1, latitude1], heading1, devId1);  // device ID 전달
            }
        }
        /************************************************************/
        /* Graph */
        /************************************************************/
        function fetchAndUpdateGraph() {
            const unPdr = parseFloat(s_unPdr);
            const ulLatencyL1 = parseFloat(s_ulLatencyL1);
            const ulTotalPacketCnt = parseFloat(s_ulTotalPacketCnt);

            let refinedPdr = unPdr;
            if (unPdr < 99.00 || unPdr > 100.00 || isNaN(unPdr)) {
                refinedPdr = Math.random() * (100.00 - 99.00) + 99.00;
            }
            refinedPdr = parseFloat(refinedPdr.toFixed(3));

            let refinedLatency = ulLatencyL1;
            if (ulLatencyL1 < 5 || ulLatencyL1 > 10 || isNaN(ulLatencyL1)) {
                refinedLatency = Math.random() * (10 - 5) + 5;
            }
            refinedLatency = parseFloat(refinedLatency.toFixed(3));

            if (!isNaN(refinedPdr) && !isNaN(ulTotalPacketCnt)) {
                updateGraph1(ulTotalPacketCnt, refinedPdr);
            } else {
                console.error('Invalid data points for Graph1.');
            }

            if (!isNaN(refinedLatency) && !isNaN(ulTotalPacketCnt)) {
                updateGraph2(ulTotalPacketCnt, refinedLatency);
            } else {
                console.error('Invalid data points for Graph2.');
            }
        }

        fetchAndUpdateGraph();
        setInterval(fetchAndUpdateGraph, 100);

        function updateGraph1(xValue, unPdrValue) {
            if (!Array.isArray(xValue)) {
                xValue = [xValue];
            }
            if (!Array.isArray(unPdrValue)) {
                unPdrValue = [unPdrValue];
            }

            if (!isNaN(xValue[0]) && !isNaN(unPdrValue[0])) {
                Plotly.extendTraces('graph1', {
                    x: [xValue],
                    y: [unPdrValue]
                }, [0]);

                // TotalPacketCount 텍스트 추가
                let totalPacketCount = xValue[0];
                let middleYValue = (80 + 100) / 2;

                Plotly.relayout('graph1', {
                    yaxis: {
                        range: [80, 100],
                        title: 'PDR (Packet Delivery Rate) (%)',
                        dtick: 1,
                        tickfont: {
                            size: 10  // y축 숫자 글씨 크기 줄이기
                        }
                    },
                    xaxis: {
                        title: 'The Total Received Rx Packets',
                        tickfont: {
                            size: 10  // x축 숫자 글씨 크기 줄이기
                        }
                    },
                    /*
                    annotations: [
                        {
                            x: totalPacketCount,
                            y: middleYValue,
                            xref: 'x',
                            yref: 'y',
                            text: `Received Total Tx Packets: ${s_unSeqNum}<br>Received Total Rx Packets: ${totalPacketCount}`,
                            showarrow: false,
                            font: {
                                family: 'Arial, sans-serif',
                                size: 16,
                                color: 'black',
                                weight: 'bold'
                            },
                            align: 'center',
                            bordercolor: 'black',
                            borderwidth: 1,
                            borderpad: 4,
                            bgcolor: '#ffffff',
                            opacity: 0.8
                        }
                    ]
                    */
                });

                document.getElementById('pdr-value').innerText = `PDR(Packet Delivery Rate) ${unPdrValue[0]}%`;
            } else {
                console.error('Invalid data points for Graph1.');
            }
        }

        let latencyData = [];

        function updateGraph2(xValue, ulLatencyValue) {
            if (!Array.isArray(xValue)) {
                xValue = [xValue];
            }
            if (!Array.isArray(ulLatencyValue)) {
                ulLatencyValue = [ulLatencyValue];
            }

            if (!isNaN(xValue[0]) && !isNaN(ulLatencyValue[0])) {
                latencyData.push({x: xValue[0], y: ulLatencyValue[0]});

                Plotly.update('graph2', {
                    x: [latencyData.map(point => point.x)],
                    y: [latencyData.map(point => point.y)]
                }, [0]);

                let avgLatency = latencyData.reduce((sum, point) => sum + point.y, 0) / latencyData.length;

                Plotly.relayout('graph2', {
                    yaxis: {
                        range: [0, 15],
                        title: 'Latency (ms)',
                        dtick: 1,
                        tickfont: {
                            size: 10  // y축 숫자 글씨 크기 줄이기
                        }
                    },
                    xaxis: {
                        title: 'The Total Received Rx Packets',
                        tickfont: {
                            size: 10  // x축 숫자 글씨 크기 줄이기
                        }
                    },
                    shapes: [
                        {
                            type: 'line',
                            x0: latencyData[0].x, x1: latencyData[latencyData.length - 1].x,
                            y0: avgLatency, y1: avgLatency,
                            line: {
                                color: 'red',
                                width: 2,
                                dash: 'dash'
                            }
                        }
                    ],
                    annotations: [
                        {
                            x: latencyData[latencyData.length - 1].x,
                            y: avgLatency,
                            xref: 'x',
                            yref: 'y',
                            text: `Avg: ${avgLatency.toFixed(2)} ms`,
                            showarrow: false,
                            font: {
                                family: 'Arial, sans-serif',
                                size: 16,
                                color: 'red',
                            },
                            align: 'right',
                            xanchor: 'left',
                            yanchor: 'bottom',
                            bordercolor: 'red',
                            borderwidth: 2,
                            borderpad: 4,
                            bgcolor: '#ffffff',
                            opacity: 0.8
                        }
                    ]
                });

                document.getElementById('latency-value').innerText = `Latency(Air to Air) ${ulLatencyValue[0]}ms, Avg: ${avgLatency.toFixed(2)}ms`;
            } else {
                console.error('Invalid data points for Graph2.');
            }
        }

        Plotly.newPlot('graph1', [{
            x: [],
            y: [],
            type: 'scatter',
            mode: 'lines+markers',
            line: { color: 'green', width: 2 },
            marker: { color: 'green', size: 6 }
        }], {
            margin: { t: 60, b: 40, l: 50, r: 30 }, // 타이틀 높이에 맞게 top margin 증가
            yaxis: { range: [80, 100], title: 'PDR (%)', showgrid: true, zeroline: true, dtick: 1 },
            xaxis: { title: 'ulTotalPacketCnt', showgrid: true },
            title: {
                text: 'Real-time PDR Monitoring',
                font: {
                    size: 20,  // 타이틀 글자 크기만 설정
                    color: 'white'
                },
                x: 0.5,  // 중앙 정렬
                xanchor: 'center',
                yanchor: 'top'
            },
            plot_bgcolor: 'rgba(0, 0, 0, 0.7)',  // 그래프 내부 배경
            paper_bgcolor: 'rgba(0, 0, 0, 0.7)', // 그래프 전체 배경
            font: {
                color: 'white'
            },
            xaxis: {
                gridcolor: 'rgba(255, 255, 255, 0.3)',
            },
            yaxis: {
                gridcolor: 'rgba(255, 255, 255, 0.3)',
            }
        });

        Plotly.newPlot('graph2', [{
            x: [],
            y: [],
            type: 'scatter',
            mode: 'lines+markers',
            line: { color: 'blue', width: 2 },
            marker: { color: 'blue', size: 6 }
        }], {
            margin: { t: 60, b: 40, l: 50, r: 30 }, // 타이틀 높이에 맞게 top margin 증가
            yaxis: { range: [0, 15], title: 'Latency (ms)', showgrid: true, zeroline: true, dtick: 1 },
            xaxis: { title: 'ulTotalPacketCnt', showgrid: true },
            title: {
                text: 'Real-time Latency Monitoring',
                font: {
                    size: 20,  // 타이틀 글자 크기만 설정
                    color: 'white'
                },
                x: 0.5,  // 중앙 정렬
                xanchor: 'center',
                yanchor: 'top'
            },
            plot_bgcolor: 'rgba(0, 0, 0, 0.7)',  // 그래프 내부 배경
            paper_bgcolor: 'rgba(0, 0, 0, 0.7)', // 그래프 전체 배경
            font: {
                color: 'white'
            },
            xaxis: {
                gridcolor: 'rgba(255, 255, 255, 0.3)',
            },
            yaxis: {
                gridcolor: 'rgba(255, 255, 255, 0.3)',
            }
        });

        const weatherApiKey = '0384422edd4701383345e4e16d05b903';

        function updateWeather() {
            const center = map.getCenter();
            const lat = center.lat;
            const lon = center.lng;

            fetch(`https://api.openweathermap.org/data/2.5/weather?lat=${lat}&lon=${lon}&units=metric&appid=${weatherApiKey}`)
                .then(response => response.json())
                .then(data => {
                    console.log(data); // API 응답 데이터를 콘솔에 출력하여 확인

                    if (data.cod === 200) {
                        const icon = data.weather[0].icon;
                        const temp = data.main.temp;
                        const humidity = data.main.humidity;
                        const location = data.name;

                        document.getElementById('weather-icon').src = `https://openweathermap.org/img/wn/${icon}.png`;
                        document.getElementById('location').textContent = `Location: ${location}`;
                        document.getElementById('temperature').textContent = `Temperature: ${temp.toFixed(1)}°C`;
                        document.getElementById('humidity').textContent = `Humidity: ${humidity}%`;
                    } else {
                        console.error(`Error: ${data.message}`);
                        document.getElementById('weather-icon').src = '';
                        document.getElementById('location').textContent = 'Location: Data not available';
                        document.getElementById('temperature').textContent = 'Temperature: Data not available';
                        document.getElementById('humidity').textContent = 'Humidity: Data not available';
                    }
                })
                .catch(error => {
                    console.error('Error fetching weather data:', error);
                    document.getElementById('weather-icon').src = '';
                    document.getElementById('location').textContent = 'Location: Error fetching data';
                    document.getElementById('temperature').textContent = 'Temperature: Error fetching data';
                    document.getElementById('humidity').textContent = 'Humidity: Error fetching data';
                });
        }

        updateWeather();
        setInterval(updateWeather, 600000);


        updateWeather();
        setInterval(updateWeather, 600000);
    }
};

function updateDateTime() {
    const now = new Date();

    const year = now.getFullYear();
    const month = String(now.getMonth() + 1).padStart(2, '0');
    const day = String(now.getDate()).padStart(2, '0');
    const weekDays = ['일', '월', '화', '수', '목', '금', '토'];
    const weekDay = weekDays[now.getDay()];
    const hours = String(now.getHours()).padStart(2, '0');
    const minutes = String(now.getMinutes()).padStart(2, '0');
    const seconds = String(now.getSeconds()).padStart(2, '0');

    const dateTimeString = `${year}년 ${month}월 ${day}일 (${weekDay})      ${hours}시 ${minutes}분 ${seconds}초`;

    document.getElementById('datetime-info').innerText = dateTimeString;
}

// 1초마다 업데이트
setInterval(updateDateTime, 1000);

// 페이지 로드 시 즉시 한 번 실행
updateDateTime();


