// SortCS, Version=1.1.0.0, Culture=neutral, PublicKeyToken=null
// SortCS.EnhanceSortTracker
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using HungarianAlgorithm;
using Microsoft.Extensions.Logging;
using SortCS;
using SortCS.Kalman;

public class EnhanceSortTracker : ITracker
{
	private readonly Dictionary<int, (Track Track, KalmanBoxTracker Tracker)> _trackers;

	private readonly ILogger<SortTracker> _logger;

	private int _trackerIndex = 1;

	public TimeSpan TrackMissTimeout { get; set; } = TimeSpan.FromMinutes(1L);

	public float IouThreshold { get; private init; }

	public int MaxMisses { get; private init; }

	public EnhanceSortTracker(float iouThreshold = 0.3f, int maxMisses = 3)
	{
		_trackers = new Dictionary<int, (Track, KalmanBoxTracker)>();
		IouThreshold = iouThreshold;
		MaxMisses = maxMisses;
	}

	public EnhanceSortTracker(ILogger<SortTracker> logger, float iouThreshold = 0.3f, int maxMisses = 3)
		: this(iouThreshold, maxMisses)
	{
		_logger = logger;
	}

	public IEnumerable<Track> Track(IEnumerable<RectangleF> boxes)
	{
		throw new NotImplementedException();
	}

	public IEnumerable<Track> Track(IEnumerable<RectangleF> boxes, TimeSpan rotationTime)
	{
		Dictionary<int, RectangleF> predictions = new Dictionary<int, RectangleF>();
		Dictionary<int, (Track Track, KalmanBoxTracker Tracker)> notInPredications = new Dictionary<int, (Track, KalmanBoxTracker)>();
		foreach (KeyValuePair<int, (Track, KalmanBoxTracker)> tracker in _trackers)
		{
			RectangleF? prediction = tracker.Value.Item2.Predict(rotationTime * 0.8);
			if (prediction.HasValue)
			{
				predictions.Add(tracker.Key, prediction.Value);
			}
			else
			{
				notInPredications.Add(tracker.Key, tracker.Value);
			}
		}
		RectangleF[] boxesArray = boxes.ToArray();
		(Dictionary<int, RectangleF> Matched, ICollection<RectangleF> Unmatched) tuple = MatchDetectionsWithPredictions(boxesArray, predictions.Values);
		Dictionary<int, RectangleF> matchedBoxes = tuple.Matched;
		ICollection<RectangleF> unmatchedBoxes = tuple.Unmatched;
		HashSet<int> activeTrackids = new HashSet<int>();
		foreach (KeyValuePair<int, RectangleF> item in matchedBoxes)
		{
			KeyValuePair<int, RectangleF> prediction2 = predictions.ElementAt(item.Key);
			(Track, KalmanBoxTracker) track = _trackers[prediction2.Key];
			track.Item1.History.Add(item.Value);
			track.Item1.Misses = 0;
			track.Item1.State = TrackState.Active;
			track.Item2.Update(item.Value);
			track.Item1.Prediction = prediction2.Value;
			track.Item1.UpdatedAt = DateTime.Now;
			track.Item2.LastPredicateAt = DateTime.Now;
			activeTrackids.Add(track.Item1.TrackId);
		}
		IEnumerable<Track> missedTracks = from x in _trackers
			where !notInPredications.ContainsKey(x.Key) && !activeTrackids.Contains(x.Key) && DateTime.Now.Subtract(x.Value.Track.UpdatedAt) > TrackMissTimeout
			select x.Value.Track;
		foreach (Track missedTrack in missedTracks)
		{
			missedTrack.Misses++;
			missedTrack.TotalMisses++;
			missedTrack.State = TrackState.Ending;
		}
		List<KeyValuePair<int, (Track, KalmanBoxTracker)>> toRemove = _trackers.Where<KeyValuePair<int, (Track, KalmanBoxTracker)>>((KeyValuePair<int, (Track Track, KalmanBoxTracker Tracker)> x) => x.Value.Track.Misses > MaxMisses).ToList();
		foreach (KeyValuePair<int, (Track, KalmanBoxTracker)> tr in toRemove)
		{
			tr.Value.Item1.State = TrackState.Ended;
			_trackers.Remove(tr.Key);
		}
		foreach (RectangleF unmatchedBox in unmatchedBoxes)
		{
			Track track2 = new Track
			{
				TrackId = _trackerIndex++,
				History = new List<RectangleF> { unmatchedBox },
				Misses = 0,
				State = TrackState.Started,
				TotalMisses = 0,
				Prediction = unmatchedBox,
				UpdatedAt = DateTime.Now
			};
			_trackers.Add(track2.TrackId, (track2, new KalmanBoxTracker(unmatchedBox)));
		}
		IEnumerable<Track> result = _trackers.Select<KeyValuePair<int, (Track, KalmanBoxTracker)>, Track>((KeyValuePair<int, (Track Track, KalmanBoxTracker Tracker)> x) => x.Value.Track).Concat(toRemove.Select<KeyValuePair<int, (Track, KalmanBoxTracker)>, Track>((KeyValuePair<int, (Track Track, KalmanBoxTracker Tracker)> y) => y.Value.Track));
		Log(result);
		return result;
	}

	private void Log(IEnumerable<Track> tracks)
	{
		if (_logger == null || !tracks.Any())
		{
			return;
		}
		IEnumerable<Track> tracksWithHistory = tracks.Where((Track x) => x.History != null);
		int longest = tracksWithHistory.Max((Track x) => x.History.Count);
		bool anyStarted = tracksWithHistory.Any((Track x) => x.History.Count == 1 && x.Misses == 0);
		int ended = tracks.Count((Track x) => x.State == TrackState.Ended);
		if (anyStarted || ended > 0)
		{
			IEnumerable<string> tracksStr = tracks.Select((Track x) => $"{x.TrackId}{((x.State == TrackState.Active) ? null : $": {x.State}")}");
			_logger.LogDebug("Tracks: [{Tracks}], Longest: {Longest}, Ended: {Ended}", string.Join(",", tracksStr), longest, ended);
		}
	}

	private (Dictionary<int, RectangleF> Matched, ICollection<RectangleF> Unmatched) MatchDetectionsWithPredictions(RectangleF[] boxes, ICollection<RectangleF> trackPredictions)
	{
		if (trackPredictions.Count == 0)
		{
			return (Matched: new Dictionary<int, RectangleF>(), Unmatched: boxes);
		}
		int[,] matrix = new int[boxes.Length, trackPredictions.Count];
		RectangleF[] trackPredictionsArray = trackPredictions.ToArray();
		for (int i = 0; i < boxes.Length; i++)
		{
			for (int j = 0; j < trackPredictionsArray.Length; j++)
			{
				matrix[i, j] = (int)(-100f * IoU(boxes[i], trackPredictionsArray[j]));
			}
		}
		if (boxes.Length > trackPredictions.Count)
		{
			int[] extra = new int[boxes.Length - trackPredictions.Count];
			matrix = Enumerable.Range(0, boxes.Length).SelectMany((int row) => (from col in Enumerable.Range(0, trackPredictions.Count)
				select matrix[row, col]).Concat(extra)).ToArray(boxes.Length, boxes.Length);
		}
		int[,] original = (int[,])matrix.Clone();
		int minimalThreshold = (int)((0f - IouThreshold) * 100f);
		Dictionary<int, int> boxTrackerMapping = (from bt in matrix.FindAssignments().Select((int ti, int bi) => (bi: bi, ti: ti))
			where bt.ti < trackPredictions.Count && original[bt.bi, bt.ti] <= minimalThreshold
			select bt).ToDictionary(((int bi, int ti) bt) => bt.bi, ((int bi, int ti) bt) => bt.ti);
		RectangleF[] unmatchedBoxes = boxes.Where((RectangleF _, int index) => !boxTrackerMapping.ContainsKey(index)).ToArray();
		int value;
		Dictionary<int, RectangleF> matchedBoxes = (from tb in boxes.Select((RectangleF box, int index) => boxTrackerMapping.TryGetValue(index, out value) ? (Tracker: value, Box: box) : (Tracker: -1, Box: RectangleF.Empty))
			where tb.Tracker != -1
			select tb).ToDictionary(((int Tracker, RectangleF Box) tb) => tb.Tracker, ((int Tracker, RectangleF Box) tb) => tb.Box);
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
}
