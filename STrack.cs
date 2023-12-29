using MathNet.Numerics.LinearAlgebra;

namespace ByteTrackCSharp
{
    public enum STrackState
    {
        New = 0,
        Tracked = 1,
        Lost = 2,
        Removed = 3,
    };

    public class STrack
    {
        KalmanFilter kalman_filter_;
        Matrix<float> mean_;
        Matrix<float> covariance_;

        Rect rect_;
        STrackState state_;

        bool is_activated_;
        float score_;

        public STrack(Rect rect, float score)
        {
            kalman_filter_ = new KalmanFilter();
            mean_ = MatrixUtil.StateMean();
            covariance_ = MatrixUtil.StateCov();
            rect_ = rect;
            state_ = STrackState.New;
            is_activated_ = false;
            score_ = score;
            TrackId = 0;
            FrameId = 0;
            StartFrameId = 0;
            TrackletLength = 0;
        }
        public Rect getRect() => rect_;

        public STrackState getSTrackState() => state_;

        public bool isActivated() => is_activated_;
        public float getScore() => score_;

        public int TrackId { get; private set; }

        public int FrameId { get; private set; }

        public int StartFrameId { get; private set; }

        public int TrackletLength { get; private set; }

        public void activate(int frame_id, int track_id)
        {
            activateKalmanFilters();

            updateRect();

            state_ = STrackState.Tracked;
            if (frame_id == 1)
            {
                is_activated_ = true;
            }
            TrackId = track_id;
            FrameId = frame_id;
            StartFrameId = frame_id;
            TrackletLength = 0;
        }

        public void reActivate(STrack new_track, int frame_id, int new_track_id = -1)
        {
            updateKalmanFilters(new_track);
            updateRect();

            state_ = STrackState.Tracked;
            is_activated_ = true;
            score_ = new_track.getScore();
            if (0 <= new_track_id)
            {
                TrackId = new_track_id;
            }
            FrameId = frame_id;
            TrackletLength = 0;
        }

        public virtual void predict()
        {
            if (state_ != STrackState.Tracked)
            {
                mean_[0, 7] = 0;
            }
            kalman_filter_.predict(ref mean_, ref covariance_);
        }

        public virtual void update(STrack new_track, int frame_id)
        {
            updateKalmanFilters(new_track);
            updateRect();

            state_ = STrackState.Tracked;
            is_activated_ = true;
            score_ = new_track.getScore();
            FrameId = frame_id;
            TrackletLength++;
        }

        public void markAsLost()
        {
            state_ = STrackState.Lost;
        }

        public void markAsRemoved()
        {
            state_ = STrackState.Removed;
        }

        public void updateRect()
        {
            rect_.setWidth(mean_[0, 2] * mean_[0, 3]);
            rect_.SetHeight(mean_[0, 3]);
            rect_.setX(mean_[0, 0] - rect_.width() / 2);
            rect_.setY(mean_[0, 1] - rect_.height() / 2);
        }

        protected virtual void activateKalmanFilters()
        {
            kalman_filter_.initiate(ref mean_, ref covariance_, rect_.getXyah());
        }

        protected virtual void updateKalmanFilters(STrack new_track)
        {
            kalman_filter_.update(ref mean_, ref covariance_, new_track.getRect().getXyah());
        }
    }
}