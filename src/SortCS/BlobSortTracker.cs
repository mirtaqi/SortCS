using HungarianAlgorithm;
using Microsoft.Extensions.Logging;
using SortCS.Kalman;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace SortCS;
public class BlobSortTracker : ITracker
{

    public event EventHandler<(EnhancedTrack track, ListChangedType changedType)> TrackChanged;
    private readonly Dictionary<int, (EnhancedTrack Track, KalmanBoxTracker Tracker)> _trackers;

    private readonly ILogger<SortTracker> _logger;

    private int _trackerIndex = 1;

    public TimeSpan TrackMissTimeout { get; set; } = TimeSpan.FromMinutes(1L);

    public float IouThreshold { get; private init; }

    public int MaxMisses { get; private init; }

    public BlobSortTracker(float iouThreshold = 0.3f, int maxMisses = 3)
    {
        _trackers = new Dictionary<int, (EnhancedTrack, KalmanBoxTracker)>();
        IouThreshold = iouThreshold;
        MaxMisses = maxMisses;
    }

    public BlobSortTracker(ILogger<SortTracker> logger, float iouThreshold = 0.3f, int maxMisses = 3)
        : this(iouThreshold, maxMisses)
    {
        _logger = logger;
    }

    public IEnumerable<Track> Track(IEnumerable<RectangleF> boxes)
    {
        throw new NotImplementedException();
    }

    public IEnumerable<EnhancedTrack> Track(IEnumerable<RectangleModel> boxes, TimeSpan rotationTime,double initialIou,double iou)
    {
        Dictionary<int, (RectangleF rect, EnhancedTrack track)> predictions = new Dictionary<int, (RectangleF rect, EnhancedTrack track)>();
        Dictionary<int, (EnhancedTrack Track, KalmanBoxTracker Tracker)> notInPredications = new Dictionary<int, (EnhancedTrack, KalmanBoxTracker)>();
        foreach (KeyValuePair<int, (EnhancedTrack, KalmanBoxTracker)> tracker in _trackers)
        {
            //RectangleF? prediction = tracker.Value.Item2.Predict();
            RectangleF? prediction = tracker.Value.Item2.Predict(rotationTime * 0.8);
            if (prediction.HasValue)
            {
                predictions.Add(tracker.Key, (prediction.Value,tracker.Value.Item1));
            }
            else
            {
                notInPredications.Add(tracker.Key, tracker.Value);
            }
        }
        var boxesArray = boxes.ToArray();
        (Dictionary<int, RectangleModel?> Matched, ICollection<RectangleModel?> Unmatched) tuple = MatchDetectionsWithPredictions(boxesArray, predictions.Values,initialIou,iou);
        Dictionary<int, RectangleModel?> matchedBoxes = tuple.Matched;
        ICollection<RectangleModel> unmatchedBoxes = tuple.Unmatched;
        HashSet<int> activeTrackids = new HashSet<int>();
        foreach (KeyValuePair<int, RectangleModel?> item in matchedBoxes)
        {
            if(item.Key<0 || item.Value is null)
                continue;
            KeyValuePair<int, RectangleF> prediction2 =new KeyValuePair<int, RectangleF>(predictions.ElementAt(item.Key).Key,predictions.ElementAt(item.Key).Value.rect);
            (EnhancedTrack, KalmanBoxTracker) track = _trackers[prediction2.Key];
            track.Item1.History.Add(item.Value);
            track.Item1.Misses = 0;
            track.Item1.State = TrackState.Active;
            track.Item2.Update(item.Value.Box);
            track.Item1.Prediction = prediction2.Value;
            track.Item1.UpdatedAt = DateTime.Now;
            track.Item2.LastPredicateAt = DateTime.Now;
            track.Item1.IsDirty = true;
            activeTrackids.Add(track.Item1.TrackId);
            OnTrackChanged((track.Item1,ListChangedType.ItemChanged));
        }
        IEnumerable<EnhancedTrack> missedTracks = from x in _trackers
                                          where !notInPredications.ContainsKey(x.Key) && !activeTrackids.Contains(x.Key) && DateTime.Now.Subtract(x.Value.Track.UpdatedAt) > TrackMissTimeout
                                          select x.Value.Track;
        foreach (EnhancedTrack missedTrack in missedTracks)
        {
            var previousMisses = missedTrack.Misses;
            var timeDiff = DateTime.Now.Subtract(missedTrack.UpdatedAt);
            var calculatedMiss = (int)(timeDiff / rotationTime);
            var missDif = calculatedMiss - previousMisses;
            if (calculatedMiss > 0)
            {
                missedTrack.Misses = calculatedMiss;
                missedTrack.TotalMisses += missDif;
                missedTrack.State = TrackState.Ending;
                missedTrack.IsDirty = true;
                OnTrackChanged((missedTrack,ListChangedType.ItemChanged));
            }
        }
        List<KeyValuePair<int, (EnhancedTrack, KalmanBoxTracker)>> toRemove = _trackers.Where<KeyValuePair<int, (EnhancedTrack, KalmanBoxTracker)>>((KeyValuePair<int, (EnhancedTrack Track, KalmanBoxTracker Tracker)> x) => x.Value.Track.Misses > MaxMisses).ToList();
        foreach (KeyValuePair<int, (EnhancedTrack, KalmanBoxTracker)> tr in toRemove)
        {
            tr.Value.Item1.State = TrackState.Ended;
            _trackers.Remove(tr.Key);
            OnTrackChanged((tr.Value.Item1, ListChangedType.ItemDeleted));
        }
        foreach (var unmatchedBox in unmatchedBoxes)
        {
            EnhancedTrack track2 = new EnhancedTrack
            {
                TrackId = _trackerIndex++,
                History = new List<RectangleModel> { unmatchedBox },
                Misses = 0,
                State = TrackState.Started,
                TotalMisses = 0,
                Prediction = unmatchedBox.Box,
                UpdatedAt = DateTime.Now,
                IsDirty = true
            };
            _trackers.Add(track2.TrackId, (track2, new KalmanBoxTracker(unmatchedBox.Box)));
            OnTrackChanged((track2,ListChangedType.ItemAdded));
        }
        IEnumerable<EnhancedTrack> result = _trackers.Select<KeyValuePair<int, (EnhancedTrack, KalmanBoxTracker)>, EnhancedTrack>((KeyValuePair<int, (EnhancedTrack Track, KalmanBoxTracker Tracker)> x) => x.Value.Track).Concat(toRemove.Select<KeyValuePair<int, (EnhancedTrack, KalmanBoxTracker)>, EnhancedTrack>((KeyValuePair<int, (EnhancedTrack Track, KalmanBoxTracker Tracker)> y) => y.Value.Track));
        Log(result);
        return result;
    }

    private void Log(IEnumerable<EnhancedTrack> tracks)
    {
        if (_logger == null || !tracks.Any())
        {
            return;
        }
        IEnumerable<EnhancedTrack> tracksWithHistory = tracks.Where((EnhancedTrack x) => x.History != null);
        int longest = tracksWithHistory.Max((EnhancedTrack x) => x.History.Count);
        bool anyStarted = tracksWithHistory.Any((EnhancedTrack x) => x.History.Count == 1 && x.Misses == 0);
        int ended = tracks.Count((EnhancedTrack x) => x.State == TrackState.Ended);
        if (anyStarted || ended > 0)
        {
            IEnumerable<string> tracksStr = tracks.Select((EnhancedTrack x) => $"{x.TrackId}{((x.State == TrackState.Active) ? null : $": {x.State}")}");
            _logger.LogDebug("Tracks: [{Tracks}], Longest: {Longest}, Ended: {Ended}", string.Join(",", tracksStr), longest, ended);
        }
    }

    private (Dictionary<int, RectangleModel?> Matched, ICollection<RectangleModel> Unmatched) MatchDetectionsWithPredictions(RectangleModel[] boxes, ICollection<(RectangleF rect,EnhancedTrack track)> trackPredictions, double initialIou, double iou)
    {
        if (trackPredictions.Count == 0)
        {
            return (Matched: new Dictionary<int, RectangleModel?>(), Unmatched: boxes);
        }
        int[,] matrix = new int[boxes.Length, trackPredictions.Count];
        var trackPredictionsArray = trackPredictions.ToArray();
        for (int i = 0; i < boxes.Length; i++)
        {
            for (int j = 0; j < trackPredictionsArray.Length; j++)
            {
                matrix[i, j] = (int)(-100f * IoU(boxes[i].Box, trackPredictionsArray[j].rect));
            }
        }
        if (boxes.Length > trackPredictions.Count)
        {
            int[] extra = new int[boxes.Length - trackPredictions.Count];
            matrix = Enumerable.Range(0, boxes.Length).SelectMany((int row) => (from col in Enumerable.Range(0, trackPredictions.Count)
                                                                                select matrix[row, col]).Concat(extra)).ToArray(boxes.Length, boxes.Length);
        }

        var getIotThreshold = (int index) =>
        {
            var v= trackPredictionsArray[index].track.IsConfirmed?iou:initialIou;
            return (int)((0f - v) * 100f);
        };
        int[,] original = (int[,])matrix.Clone();
        int minimalThreshold = (int)((0f - IouThreshold) * 100f);
        Dictionary<int, int> boxTrackerMapping = (from bt in matrix.FindAssignments().Select((int ti, int bi) => (bi: bi, ti: ti))
                                                  where bt.ti < trackPredictions.Count && original[bt.bi, bt.ti] <=  getIotThreshold(bt.ti)
                                                  select bt).ToDictionary(((int bi, int ti) bt) => bt.bi, ((int bi, int ti) bt) => bt.ti);
        RectangleModel[] unmatchedBoxes = boxes.Where((RectangleModel _, int index) => !boxTrackerMapping.ContainsKey(index)).ToArray();
        int value;
        Dictionary<int, RectangleModel?> matchedBoxes = (from tb in boxes.Select((RectangleModel box, int index) => boxTrackerMapping.TryGetValue(index, out value) ? (Tracker: value, Box: box) : (Tracker: -1, Box: (RectangleModel?)null))
                                                    where tb.Tracker != -1
                                                    select tb).ToDictionary(((int Tracker, RectangleModel? Box) tb) => tb.Tracker, ((int Tracker, RectangleModel? Box) tb) => tb.Box);
        return (Matched: matchedBoxes, Unmatched: unmatchedBoxes);
    }

    private static float IoU(RectangleF a, RectangleF b)
    {
        RectangleF intersection = RectangleF.Intersect(a, b);
        if (intersection.IsEmpty)
        {
            return 0f;
        }
        float intersectArea = (1f + intersection.Width) * (1f + intersection.Height);
        float unionArea = (1f + a.Width) * (1f + a.Height) + (1f + b.Width) * (1f + b.Height) - intersectArea;
        return intersectArea / (unionArea + 1E-05f);
    }

    protected virtual void OnTrackChanged((EnhancedTrack track, ListChangedType changedType) e)
    {
        TrackChanged?.Invoke(this, e);
    }
}
