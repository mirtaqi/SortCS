using System;
using System.Drawing;

namespace SortCS.Kalman;

internal class KalmanBoxTracker
{
    private static readonly Matrix _stateTransitioningMatrix = new(
        new float[,]
        {
            { 1, 0, 0, 0, 1, 0, 0 },
            { 0, 1, 0, 0, 0, 1, 0 },
            { 0, 0, 1, 0, 0, 0, 1 },
            { 0, 0, 0, 1, 0, 0, 0 },
            { 0, 0, 0, 0, 1, 0, 0 },
            { 0, 0, 0, 0, 0, 1, 0 },
            { 0, 0, 0, 0, 0, 0, 1 }
        });

    private static readonly Matrix _measurementFunction = new(
        new float[,]
        {
            { 1, 0, 0, 0, 0, 0, 0 },
            { 0, 1, 0, 0, 0, 0, 0 },
            { 0, 0, 1, 0, 0, 0, 0 },
            { 0, 0, 0, 1, 0, 0, 0 }
        });

    private static readonly Matrix _uncertaintyCovariances = new(
        new float[,]
        {
            { 10, 0, 0, 0, 0, 0, 0 },
            { 0, 10, 0, 0, 0, 0, 0 },
            { 0, 0, 10, 0, 0, 0, 0 },
            { 0, 0, 0, 10, 0, 0, 0 },
            { 0, 0, 0, 0, 10000, 0, 0 },
            { 0, 0, 0, 0, 0, 10000, 0 },
            { 0, 0, 0, 0, 0, 0, 10000 }
        });

    private static readonly Matrix _measurementUncertainty = new(new float[,]
    {
        { 1, 0, 0, 0 },
        { 0, 1, 0, 0 },
        { 0, 0, 10, 0 },
        { 0, 0, 0, 10 },
    });

    private static readonly Matrix _processUncertainty = new(
        new float[,]
        {
            { 1, 0, 0, 0, 0, 0, 0 },
            { 0, 1, 0, 0, 0, 0, 0 },
            { 0, 0, 1, 0, 0, 0, 0 },
            { 0, 0, 0, 1, 0, 0, 0 },
            { 0, 0, 0, 0, .01f, 0, 0 },
            { 0, 0, 0, 0, 0, .01f, 0 },
            { 0, 0, 0, 0, 0, 0, .0001f }
        });

    private readonly KalmanFilter _filter;
    public DateTime? LastPredicateAt { get; set; }

    public RectangleF? LastPredication { get; private set; }

    public KalmanBoxTracker(RectangleF box)
    {
        _filter = new KalmanFilter(7, 4)
        {
            StateTransitionMatrix = _stateTransitioningMatrix,
            MeasurementFunction = _measurementFunction,
            UncertaintyCovariances = _uncertaintyCovariances,
            MeasurementUncertainty = _measurementUncertainty,
            ProcessUncertainty = _processUncertainty,
            CurrentState = ToMeasurement(box).Append(0, 0, 0)
        };
    }

    public void Update(RectangleF box)
    {
        _filter.Update(ToMeasurement(box));
    }
    public RectangleF? Predict(TimeSpan timeThreshold)
    {
        if (LastPredicateAt.HasValue && DateTime.Now.Subtract(LastPredicateAt.Value) < timeThreshold)
        {
            return LastPredication;
        }
        LastPredicateAt = DateTime.Now;
        if (_filter.CurrentState[6] + _filter.CurrentState[2] <= 0f)
        {
            _filter.SetState(6, 0f);
        }
        _filter.Predict();
        RectangleF prediction = ToBoundingBox(_filter.CurrentState);
        LastPredication = prediction;
        return prediction;
    }

    public RectangleF Predict()
    {
        if (_filter.CurrentState[6] + _filter.CurrentState[2] <= 0)
        {
            _filter.SetState(6, 0);
        }

        _filter.Predict();

        var prediction = ToBoundingBox(_filter.CurrentState);

        return prediction;
    }

    private static Vector ToMeasurement(RectangleF box)
    {
        var center = new PointF(box.Left + (box.Width / 2f), box.Top + (box.Height / 2f));
        return new Vector(center.X, center.Y, box.Width * box.Height, box.Width / box.Height);
    }

    private static RectangleF ToBoundingBox(Vector currentState)
    {
        var w = Math.Sqrt(currentState[2] * currentState[3]);
        var h = currentState[2] / w;

        return new RectangleF(
            (float)(currentState[0] - (w / 2)),
            (float)(currentState[1] - (h / 2)),
            (float)w,
            (float)h);
    }
}