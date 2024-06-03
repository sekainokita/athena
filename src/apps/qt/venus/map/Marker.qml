// Copyright (C) 2022 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

import QtQuick
import QtLocation

//! [mqi-top]
MapQuickItem {
    id: marker
//! [mqi-top]

//! [mqi-anchor]
    anchorPoint.x: image.width/4
    anchorPoint.y: image.height

    HoverHandler {
        id: hoverHandler
    }
    TapHandler {
        id: tapHandler
        acceptedButtons: Qt.RightButton
        gesturePolicy: TapHandler.WithinBounds
        onTapped: {
            mapview.currentMarker = -1
            for (var i = 0; i< mapview.markers.length; i++){
                if (marker == mapview.markers[i]){
                    mapview.currentMarker = i
                    break
                }
            }
            mapview.showMarkerMenu(marker.coordinate)
        }
    }
    DragHandler {
        id: dragHandler
        grabPermissions: PointerHandler.CanTakeOverFromItems | PointerHandler.CanTakeOverFromHandlersOfDifferentType
    }

    sourceItem: Image {
        id: image
//! [mqi-anchor]
        source: "../resources/vehicle.png"
        sourceSize.width: 50
        sourceSize.height: 50
        opacity: hoverHandler.hovered ? 0.6 : 1.0

        Text{
            id: number
            y: image.height/2.5
            width: image.width
            color: "white"
            font.bold: true
            font.pixelSize: 11
            horizontalAlignment: Text.AlignHCenter
            Component.onCompleted: {
                text = mapview.markerCounter
            }
        }

    Text{
            id: device
            y: image.height/100
            width: image.width
            color: "black"
            font.bold: true
            font.pixelSize: 11
            horizontalAlignment: Text.AlignHCenter
            Component.onCompleted: {
                text = "^"
            }
        }

    Text{
            rotation: -parent.rotation
            id: deviceId
            y: image.height/100 - 25
            width: image.width
            color: "black"
            font.bold: true
            font.pixelSize: 11
            horizontalAlignment: Text.AlignHCenter
            Component.onCompleted: {
                text = "KETI"
            }
        }
//! [mqi-closeimage]
    }
//! [mqi-closeimage]

//! [mqi-close]
}
//! [mqi-close]
