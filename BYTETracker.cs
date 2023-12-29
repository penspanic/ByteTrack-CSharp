using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace ByteTrackCSharp;

public class BYTETracker
{
    private Func<Object, STrack> strackFactory_;
    private readonly float track_thresh_;
    private readonly float high_thresh_;
    private readonly float match_thresh_;
    private readonly int max_time_lost_;

    private int frame_id_;
    private int track_id_count_;

    private List<STrack> tracked_stracks_ = new();
    private List<STrack> lost_stracks_ = new();
    private List<STrack> removed_stracks_ = new();

    public BYTETracker(Func<Object, STrack> strackFactory, int frame_rate = 30, int track_buffer = 30, float track_thresh = 0.5f,
        float high_thresh = 0.6f, float match_thresh = 0.8f)
    {
        strackFactory_ = strackFactory;
        track_thresh_ = track_thresh;
        high_thresh_ = high_thresh;
        match_thresh_ = match_thresh;
        max_time_lost_ = (int)(frame_rate / 30f * track_buffer);
        frame_id_ = 0;
        track_id_count_ = 0;
    }

    public List<STrack> update(List<Object> objects)
    {
        ////////////////// Step 1: Get detections //////////////////
        frame_id_++;

        // Create new STracks using the result of object detection
        List<STrack> det_stracks = new();
        List<STrack> det_low_stracks = new();

        foreach (var obj in objects)
        {
            var strack = strackFactory_(obj);
            if (obj.prob >= track_thresh_)
            {
                det_stracks.Add(strack);
            }
            else
            {
                det_low_stracks.Add(strack);
            }
        }

        // Create lists of existing STrack
        List<STrack> active_stracks = new();
        List<STrack> non_active_stracks = new();

        foreach (var tracked_strack in tracked_stracks_)
        {
            if (!tracked_strack.isActivated())
            {
                non_active_stracks.Add(tracked_strack);
            }
            else
            {
                active_stracks.Add(tracked_strack);
            }
        }

        List<STrack> strack_pool = jointStracks(active_stracks, lost_stracks_);

        // Predict current pose by KF
        foreach (var strack in strack_pool)
        {
            strack.predict();
        }

        ////////////////// Step 2: First association, with IoU //////////////////
        List<STrack> current_tracked_stracks = new();
        List<STrack> remain_tracked_stracks = new();
        List<STrack> remain_det_stracks = new();
        List<STrack> refind_stracks = new();

        {
            List<List<int>> matches_idx = new();
            List<int> unmatch_detection_idx = new(), unmatch_track_idx = new();

            var dists = calcIouDistance(strack_pool, det_stracks);
            linearAssignment(dists, strack_pool.Count, det_stracks.Count, match_thresh_,
                matches_idx, unmatch_track_idx, unmatch_detection_idx);

            foreach (var match_idx in matches_idx)
            {
                var track = strack_pool[match_idx[0]];
                var det = det_stracks[match_idx[1]];
                if (track.getSTrackState() == STrackState.Tracked)
                {
                    track.update(det, frame_id_);
                    current_tracked_stracks.Add(track);
                }
                else
                {
                    track.reActivate(det, frame_id_);
                    refind_stracks.Add(track);
                }
            }

            foreach (var unmatch_idx in unmatch_detection_idx)
            {
                remain_det_stracks.Add(det_stracks[unmatch_idx]);
            }

            foreach (var unmatch_idx in unmatch_track_idx)
            {
                if (strack_pool[unmatch_idx].getSTrackState() == STrackState.Tracked)
                {
                    remain_tracked_stracks.Add(strack_pool[unmatch_idx]);
                }
            }
        }

        ////////////////// Step 3: Second association, using low score dets //////////////////
        List<STrack> current_lost_stracks = new();

        {
            List<List<int>> matches_idx = new();
            List<int> unmatch_track_idx = new(), unmatch_detection_idx = new();

            var dists = calcIouDistance(remain_tracked_stracks, det_low_stracks);
            linearAssignment(dists, remain_tracked_stracks.Count, det_low_stracks.Count, 0.5f,
                matches_idx, unmatch_track_idx, unmatch_detection_idx);

            foreach (var match_idx in matches_idx)
            {
                var track = remain_tracked_stracks[match_idx[0]];
                var det = det_low_stracks[match_idx[1]];
                if (track.getSTrackState() == STrackState.Tracked)
                {
                    track.update(det, frame_id_);
                    current_tracked_stracks.Add(track);
                }
                else
                {
                    track.reActivate(det, frame_id_);
                    refind_stracks.Add(track);
                }
            }

            foreach (var unmatch_track in unmatch_track_idx)
            {
                var track = remain_tracked_stracks[unmatch_track];
                if (track.getSTrackState() != STrackState.Lost)
                {
                    track.markAsLost();
                    current_lost_stracks.Add(track);
                }
            }
        }

        ////////////////// Step 4: Init new stracks //////////////////
        List<STrack> current_removed_stracks = new();

        {
            List<int> unmatch_detection_idx = new();
            List<int> unmatch_unconfirmed_idx = new();
            List<List<int>> matches_idx = new();

            // Deal with unconfirmed tracks, usually tracks with only one beginning frame
            var dists = calcIouDistance(non_active_stracks, remain_det_stracks);
            linearAssignment(dists, non_active_stracks.Count, remain_det_stracks.Count, 0.7f,
                matches_idx, unmatch_unconfirmed_idx, unmatch_detection_idx);

            foreach (var match_idx in matches_idx)
            {
                non_active_stracks[match_idx[0]].update(remain_det_stracks[match_idx[1]], frame_id_);
                current_tracked_stracks.Add(non_active_stracks[match_idx[0]]);
            }

            foreach (var unmatch_idx in unmatch_unconfirmed_idx)
            {
                var track = non_active_stracks[unmatch_idx];
                track.markAsRemoved();
                current_removed_stracks.Add(track);
            }

            // Add new stracks
            foreach (var unmatch_idx in unmatch_detection_idx)
            {
                var track = remain_det_stracks[unmatch_idx];
                if (track.getScore() < high_thresh_)
                {
                    continue;
                }
                track_id_count_++;
                track.activate(frame_id_, track_id_count_);
                current_tracked_stracks.Add(track);
            }
        }

        ////////////////// Step 5: Update state //////////////////
        foreach (var lost_strack in lost_stracks_)
        {
            if (frame_id_ - lost_strack.FrameId > max_time_lost_)
            {
                lost_strack.markAsRemoved();
                current_removed_stracks.Add(lost_strack);
            }
        }

        tracked_stracks_ = jointStracks(current_tracked_stracks, refind_stracks);
        lost_stracks_ = subStracks(jointStracks(subStracks(lost_stracks_, tracked_stracks_), current_lost_stracks), removed_stracks_);
        removed_stracks_ = jointStracks(removed_stracks_, current_removed_stracks);

        List<STrack> tracked_stracks_out = new(), lost_stracks_out = new();
        removeDuplicateStracks(tracked_stracks_, lost_stracks_, tracked_stracks_out, lost_stracks_out);
        tracked_stracks_ = tracked_stracks_out;
        lost_stracks_ = lost_stracks_out;

        List<STrack> output_stracks = new();
        foreach (var track in tracked_stracks_)
        {
            if (track.isActivated())
            {
                output_stracks.Add(track);
            }
        }

        return output_stracks;
    }

    public List<STrack> jointStracks(List<STrack> a_tlist, List<STrack> b_tlist)
    {
        Dictionary<int, int> exists = new();
        List<STrack> res = new();
        foreach (var aTrack in a_tlist)
        {
            int trackId = aTrack.TrackId;
            if (!exists.ContainsKey(trackId))
            {
                exists.Add(trackId, 1);
            }
            res.Add(aTrack);
        }

        foreach (var bTrack in b_tlist)
        {
            int trackId = bTrack.TrackId;
            if (!exists.ContainsKey(trackId))
            {
                exists.Add(trackId, 1);
                res.Add(bTrack);
            }
        }
        return res;
    }

    List<STrack> subStracks(List<STrack> a_tlist, List<STrack> b_tlist)
    {
        Dictionary<int, STrack> stracks = new();
        foreach (var aTrack in a_tlist)
        {
            int trackId = aTrack.TrackId;
            if (!stracks.ContainsKey(trackId))
            {

                stracks[trackId] = aTrack;
            }
        }

        foreach (var bTrack in b_tlist)
        {
            int trackId = bTrack.TrackId;
            stracks.Remove(trackId);
        }
        List<STrack> res = new();
        foreach (var pair in stracks)
        {
            res.Add(pair.Value);
        }
        return res;
    }

    public void removeDuplicateStracks(List<STrack> a_stracks, List<STrack> b_stracks, List<STrack> a_res, List<STrack> b_res)
    {
        var ious = calcIouDistance(a_stracks, b_stracks);

        List<(int, int)> overlapping_combinations = new();
        for (int i = 0; i < ious.Count; i++)
        {
            for (int j = 0; j < ious[i].Count; j++)
            {
                if (ious[i][j] < 0.15)
                {
                    overlapping_combinations.Add((i, j));
                }
            }
        }

        List<bool> a_overlapping = new List<bool>(a_stracks.Count);
        for (int i = 0; i < a_stracks.Count; ++i)
            a_overlapping.Add(false);

        List<bool> b_overlapping = new List<bool>(b_stracks.Count);
        for (int i = 0; i < b_stracks.Count; ++i)
            b_overlapping.Add(false);

        foreach (var (a_idx, b_idx) in overlapping_combinations)
        {
            int timep = a_stracks[a_idx].FrameId - a_stracks[a_idx].StartFrameId;
            int timeq = b_stracks[b_idx].FrameId - b_stracks[b_idx].StartFrameId;
            if (timep > timeq)
            {
                b_overlapping[b_idx] = true;
            }
            else
            {
                a_overlapping[a_idx] = true;
            }
        }

        for (int ai = 0; ai < a_stracks.Count; ai++)
        {
            if (!a_overlapping[ai])
            {
                a_res.Add(a_stracks[ai]);
            }
        }

        for (int bi = 0; bi < b_stracks.Count; bi++)
        {
            if (!b_overlapping[bi])
            {
                b_res.Add(b_stracks[bi]);
            }
        }
    }

    public void linearAssignment(List<List<float>> cost_matrix,
        int cost_matrix_size,
        int cost_matrix_size_size,
        float thresh,
        List<List<int>> matches,
        List<int> a_unmatched,
        List<int> b_unmatched)
    {
        if (cost_matrix.Count == 0)
        {
            for (int i = 0; i < cost_matrix_size; i++)
            {
                a_unmatched.Add(i);
            }
            for (int i = 0; i < cost_matrix_size_size; i++)
            {
                b_unmatched.Add(i);
            }
            return;
        }

        List<int> rowsol = new(); List<int> colsol = new();
        execLapjv(cost_matrix, rowsol, colsol, true, thresh);
        for (int i = 0; i < rowsol.Count; i++)
        {
            if (rowsol[i] >= 0)
            {
                List<int> match = new List<int>
                {
                    i,
                    rowsol[i]
                };
                matches.Add(match);
            }
            else
            {
                a_unmatched.Add(i);
            }
        }

        for (int i = 0; i < colsol.Count; i++)
        {
            if (colsol[i] < 0)
            {
                b_unmatched.Add(i);
            }
        }
    }

    static void FillValues<T>(List<T> list, T value, int count)
    {
        list.EnsureCapacity(list.Count + count);
        for (int i = 0; i < count; ++i)
            list.Add(value);
    }

    static void FillValues<T>(List<T> list, Func<T> valueFactory, int count)
    {
        list.EnsureCapacity(list.Count + count);
        for (int i = 0; i < count; ++i)
            list.Add(valueFactory());
    }

    List<List<float>> calcIous(List<Rect> a_rect, List<Rect> b_rect)
    {
        List<List<float>> ious = new(a_rect.Count);
        if (a_rect.Count * b_rect.Count == 0)
        {
            return ious;
        }

        for (int i = 0; i <a_rect.Count; ++i)
        {
            var list = new List<float>(b_rect.Count);
            FillValues(list, 0, b_rect.Count);
            ious.Add(list);
        }

        for (int bi = 0; bi < b_rect.Count; bi++)
        {
            for (int ai = 0; ai < a_rect.Count; ai++)
            {
                ious[ai][bi] = b_rect[bi].calcIoU(a_rect[ai]);
            }
        }
        return ious;
    }

    List<List<float>> calcIouDistance(List<STrack> a_tracks, List<STrack> b_tracks)
    {
        List<Rect> a_rects = new(), b_rects = new();
        for (int i = 0; i < a_tracks.Count; i++)
        {
            a_rects.Add(a_tracks[i].getRect());
        }

        for (int i = 0; i < b_tracks.Count; i++)
        {
            b_rects.Add(b_tracks[i].getRect());
        }

        var ious = calcIous(a_rects, b_rects);

        List<List<float>> cost_matrix = new();
        for (int i = 0; i < ious.Count; i++)
        {
            List<float> iou = new();
            for (int j = 0; j < ious[i].Count; j++)
            {
                iou.Add(1 - ious[i][j]);
            }
            cost_matrix.Add(iou);
        }

        return cost_matrix;
    }

    public double execLapjv(List<List<float>> cost, List<int> rowsol, List<int> colsol, bool extend_cost, float cost_limit, bool return_cost = true)
    {
        List<List<float>> cost_c = new(cost);

        List<List<float>> cost_c_extended = new();

        int n_rows = cost.Count;
        int n_cols = cost[0].Count;

        FillValues(rowsol, 0, n_rows);
        FillValues(colsol, 0, n_cols);

        int n = 0;
        if (n_rows == n_cols)
        {
            n = n_rows;
        }
        else
        {
            if (!extend_cost)
            {
                throw new Exception("The `extend_cost` variable should set True");
            }
        }

        if (extend_cost || cost_limit < float.MaxValue)
        {
            n = n_rows + n_cols;
            FillValues(cost_c_extended, () => new List<float>(), n);
            for (int i = 0; i < cost_c_extended.Count; i++)
            {
                FillValues(cost_c_extended[i], 0, n);
            }

            if (cost_limit < float.MaxValue)
            {
                for (int i = 0; i < cost_c_extended.Count; i++)
                {
                    for (int j = 0; j < cost_c_extended[i].Count; j++)
                    {
                        cost_c_extended[i][j] = cost_limit / 2.0f;
                    }
                }
            }
            else
            {
                float cost_max = -1;
                for (int i = 0; i < cost_c.Count; i++)
                {
                    for (int j = 0; j < cost_c[i].Count; j++)
                    {
                        if (cost_c[i][j] > cost_max)
                            cost_max = cost_c[i][j];
                    }
                }
                for (int i = 0; i < cost_c_extended.Count; i++)
                {
                    for (int j = 0; j < cost_c_extended[i].Count; j++)
                    {
                        cost_c_extended[i][j] = cost_max + 1;
                    }
                }
            }

            for (int i = n_rows; i < cost_c_extended.Count; i++)
            {
                for (int j = n_cols; j < cost_c_extended[i].Count; j++)
                {
                    cost_c_extended[i][j] = 0;
                }
            }
            for (int i = 0; i < n_rows; i++)
            {
                for (int j = 0; j < n_cols; j++)
                {
                    cost_c_extended[i][j] = cost_c[i][j];
                }
            }

            cost_c.Clear();
            cost_c.AddRange(cost_c_extended);
        }

        unsafe
        {
            double** cost_ptr = (double**)Marshal.AllocHGlobal(sizeof(nint) * n);
            for (int i = 0; i < n; i++)
                cost_ptr[i] = (double*)Marshal.AllocHGlobal(sizeof(double) * n);

            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    cost_ptr[i][j] = cost_c[i][j];
                }
            }

            int* x_c = (int*)Marshal.AllocHGlobal(sizeof(int) * n);
            int* y_c = (int*)Marshal.AllocHGlobal(sizeof(int) * n);

            int ret = lapjv.lapjv_internal(n, cost_ptr, x_c, y_c);
            if (ret != 0)
            {
                throw new Exception("The result of lapjv_internal() is invalid.");
            }

            double opt = 0.0;

            if (n != n_rows)
            {
                for (int i = 0; i < n; i++)
                {
                    if (x_c[i] >= n_cols)
                        x_c[i] = -1;
                    if (y_c[i] >= n_rows)
                        y_c[i] = -1;
                }
                for (int i = 0; i < n_rows; i++)
                {
                    rowsol[i] = x_c[i];
                }
                for (int i = 0; i < n_cols; i++)
                {
                    colsol[i] = y_c[i];
                }

                if (return_cost)
                {
                    for (int i = 0; i < rowsol.Count; i++)
                    {
                        if (rowsol[i] != -1)
                        {
                            opt += cost_ptr[i][rowsol[i]];
                        }
                    }
                }
            }
            else if (return_cost)
            {
                for (int i = 0; i < rowsol.Count; i++)
                {
                    opt += cost_ptr[i][rowsol[i]];
                }
            }

            for (int i = 0; i < n; i++)
            {
                Marshal.FreeHGlobal((nint)cost_ptr[i]);
            }
            Marshal.FreeHGlobal((nint)cost_ptr);

            Marshal.FreeHGlobal((nint)x_c);
            Marshal.FreeHGlobal((nint)y_c);

            return opt;
        }
    }
}