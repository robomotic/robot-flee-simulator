"""
Parquet exporter for the Robo Fleet Simulator.

Accumulates per-step observations in memory and writes at the end of the
simulation:
  - robots_<Model>.parquet   one file per robot model, multi-indexed by
                              (serial_number, timestamp)
  - pedestrians.parquet      flat table of all pedestrian observations
"""

from typing import Any, Dict, List, Tuple


class ParquetExporter:
    def __init__(self):
        self._robot_rows: List[Dict[str, Any]] = []
        self._ped_rows:   List[Dict[str, Any]] = []

    def record_step(
        self,
        timestamp_s:   float,
        robot_states:  List[Dict[str, Any]],
        detections:    List[Dict[str, Any]],
        collisions:    List[Tuple[str, str]],
        ped_states:    List[Dict[str, Any]],
    ):
        """Append one step's worth of observations to the in-memory buffers."""
        det_by_id    = {d['id']: d for d in detections}
        collided_ids = {rid for pair in collisions for rid in pair}

        for r in robot_states:
            det = det_by_id.get(r['id'], {})
            self._robot_rows.append({
                'serial_number':         r['serial_number'],
                'timestamp':             timestamp_s,
                'latitude':              r['latitude'],
                'longitude':             r['longitude'],
                'velocity':              r['velocity'],
                'battery':               r['battery'],
                'total_humans_detected': int(det.get('total_humans', 0)),
                'total_robots_detected': int(det.get('total_robots', 0)),
                'had_collision':         r['id'] in collided_ids,
                '_model':                r['model'],
            })

        for p in ped_states:
            self._ped_rows.append({
                'pedestrian_id': p['id'],
                'timestamp':     timestamp_s,
                'latitude':      p['latitude'],
                'longitude':     p['longitude'],
                'velocity':      p['velocity'],
                'mode':          p['mode'],
            })

    def write(self):
        """Write buffered data to parquet files in the working directory."""
        import pandas as pd

        if self._robot_rows:
            df = pd.DataFrame(self._robot_rows)
            df['total_humans_detected'] = df['total_humans_detected'].astype('int32')
            df['total_robots_detected'] = df['total_robots_detected'].astype('int32')
            df['had_collision']         = df['had_collision'].astype('bool')

            for model, group in df.groupby('_model'):
                out = (
                    group
                    .drop(columns='_model')
                    .set_index(['serial_number', 'timestamp'])
                    .sort_index()
                )
                path = f'robots_{model}.parquet'
                out.to_parquet(path)
                print(f'Written {path}  ({len(out):,} rows, '
                      f'{out.index.get_level_values("serial_number").nunique()} robots)')

        if self._ped_rows:
            ped_df = pd.DataFrame(self._ped_rows)
            path = 'pedestrians.parquet'
            ped_df.to_parquet(path, index=False)
            print(f'Written {path}  ({len(ped_df):,} rows, '
                  f'{ped_df["pedestrian_id"].nunique()} unique pedestrians)')
