type EventsPanelProps = {
  events: string[];
};

export function EventsPanel({ events }: EventsPanelProps) {
  return (
    <section className="panel-card">
      <div className="panel-section-title">Events / Feedback</div>
      <div className="event-log">
        {events.map((event) => (
          <div key={event} className="event-item">
            {event}
          </div>
        ))}
      </div>
    </section>
  );
}
