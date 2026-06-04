from __future__ import annotations

import numpy as np


class KalmanTrack:
    

    def __init__(
        self,
        track_id: int,
        center_xy: np.ndarray,
        timestamp_ns: int,
        process_noise: float = 1.0,
        measurement_noise: float = 0.8,
    ) -> None:
        self.track_id = int(track_id)
        self.x = np.array(
            [center_xy[0], center_xy[1], 0.0, 0.0],
            dtype=np.float64,
        ).reshape(4, 1)

        self.P = np.diag([2.0, 2.0, 10.0, 10.0]).astype(np.float64)

        self.q = float(process_noise)
        self.r = float(measurement_noise)

        self.last_timestamp_ns = int(timestamp_ns)
        self.age = 1
        self.hits = 1
        self.missed = 0

    @property
    def center(self) -> np.ndarray:
        return self.x[:2, 0].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[2:4, 0].copy()

    @property
    def speed(self) -> float:
        vx, vy = self.velocity
        return float(np.hypot(vx, vy))

    @property
    def direction_yaw(self) -> float:
        vx, vy = self.velocity
        return float(np.arctan2(vy, vx))

    def predict(self, timestamp_ns: int) -> None:
        dt = max((int(timestamp_ns) - self.last_timestamp_ns) * 1e-9, 1e-3)

        F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        q = self.q
        Q = q * np.array(
            [
                [dt**4 / 4, 0.0, dt**3 / 2, 0.0],
                [0.0, dt**4 / 4, 0.0, dt**3 / 2],
                [dt**3 / 2, 0.0, dt**2, 0.0],
                [0.0, dt**3 / 2, 0.0, dt**2],
            ],
            dtype=np.float64,
        )

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

        self.last_timestamp_ns = int(timestamp_ns)
        self.age += 1
        self.missed += 1

    def update(self, center_xy: np.ndarray) -> None:
        z = np.asarray(center_xy, dtype=np.float64).reshape(2, 1)

        H = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ],
            dtype=np.float64,
        )

        R = np.eye(2, dtype=np.float64) * self.r**2

        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P

        self.hits += 1
        self.missed = 0



class MultiObjectTracker:
    def __init__(
        self,
        max_match_distance: float = 3.0,
        max_missed: int = 5,
        min_hits: int = 2,
    ) -> None:
        self.max_match_distance = float(max_match_distance)
        self.max_missed = int(max_missed)
        self.min_hits = int(min_hits)

        self.tracks: list[KalmanTrack] = []
        self.next_track_id = 0

    def update(self, bboxes, timestamp_ns: int) -> list[KalmanTrack]:
        for tr in self.tracks:
            tr.predict(timestamp_ns)

        detections = [np.asarray(b.center[:2], dtype=np.float64) for b in bboxes]

        unmatched_dets = set(range(len(detections)))
        unmatched_tracks = set(range(len(self.tracks)))

        pairs = []
        for ti, tr in enumerate(self.tracks):
            for di, det in enumerate(detections):
                dist = float(np.linalg.norm(tr.center - det))
                if dist <= self.max_match_distance:
                    pairs.append((dist, ti, di))

        pairs.sort(key=lambda x: x[0])

        for _, ti, di in pairs:
            if ti not in unmatched_tracks or di not in unmatched_dets:
                continue

            self.tracks[ti].update(detections[di])
            unmatched_tracks.remove(ti)
            unmatched_dets.remove(di)

        for di in unmatched_dets:
            self.tracks.append(
                KalmanTrack(
                    track_id=self.next_track_id,
                    center_xy=detections[di],
                    timestamp_ns=timestamp_ns,
                )
            )
            self.next_track_id += 1

        self.tracks = [
            tr for tr in self.tracks
            if tr.missed <= self.max_missed
        ]

        return [
            tr for tr in self.tracks
            if tr.hits >= self.min_hits and tr.missed == 0
        ]