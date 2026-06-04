#ifndef AMR_VISUALIZATION__FILE_PROBE_HPP_
#define AMR_VISUALIZATION__FILE_PROBE_HPP_

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QString>
#include <QStringList>

namespace amr::visualization
{

struct FileProbe
{
  QString label;
  QString raw_path;
  int raw_path_length{0};
  QString cleaned_path;
  QString absolute_path;
  QString canonical_path;
  bool exists{false};
  bool is_file{false};
  bool readable{false};
  qint64 size{0};
  bool open_ok{false};
  QString error_string;
  QString reason;
};

namespace file_probe_detail
{

inline QString escaped_path_text(const QString &text)
{
  QString escaped;
  escaped.reserve(text.size());
  for (const QChar character : text) {
    if (character == '\\') {
      escaped += "\\\\";
    } else if (character == '\n') {
      escaped += "\\n";
    } else if (character == '\r') {
      escaped += "\\r";
    } else if (character == '\t') {
      escaped += "\\t";
    } else if (character == '\'') {
      escaped += "\\'";
    } else if (character.unicode() < 0x20 || character.unicode() == 0x7F) {
      escaped += QString("\\u%1").arg(character.unicode(), 4, 16, QLatin1Char('0'));
    } else {
      escaped += character;
    }
  }
  return escaped;
}

inline QString utf8_hex_dump(const QString &text)
{
  const QByteArray bytes = text.toUtf8();
  QStringList parts;
  parts.reserve(bytes.size());
  for (const char byte : bytes) {
    parts.push_back(QString("%1").arg(static_cast<unsigned char>(byte), 2, 16, QLatin1Char('0')));
  }
  return parts.join(' ');
}

}  // namespace file_probe_detail

inline FileProbe probeFilePath(const QString &label, const QString &raw_path)
{
  FileProbe probe;
  probe.label = label;
  probe.raw_path = raw_path;
  probe.raw_path_length = raw_path.size();

  const QString trimmed_path = raw_path.trimmed();
  if (trimmed_path.isEmpty()) {
    probe.reason = "empty path";
    probe.error_string = "empty path";
    return probe;
  }

  probe.cleaned_path = QDir::cleanPath(trimmed_path);
  const QFileInfo file_info(probe.cleaned_path);
  probe.absolute_path = file_info.absoluteFilePath();
  probe.canonical_path = file_info.canonicalFilePath();
  probe.exists = file_info.exists();
  probe.is_file = file_info.isFile();
  probe.readable = file_info.isReadable();
  probe.size = probe.exists ? file_info.size() : 0;

  QFile file(probe.cleaned_path);
  probe.open_ok = file.open(QIODevice::ReadOnly);
  probe.error_string = probe.open_ok ? QString() : file.errorString();
  if (probe.open_ok) {
    file.close();
  }

  if (!probe.exists) {
    probe.reason = "not found";
  } else if (!probe.is_file) {
    probe.reason = "not a file";
  } else if (!probe.readable) {
    probe.reason = "not readable";
  } else if (!probe.open_ok) {
    probe.reason = "open failed";
  } else {
    probe.reason = "ok";
  }
  return probe;
}

inline QString fileProbeToDiagnosticText(const FileProbe &probe)
{
  return QString(
    "label=%1, raw_path='%2', raw_path_length=%3, cleaned_path='%4', "
    "cleaned_path_length=%5, cleaned_path_utf8_hex=%6, cleaned_path_escaped='%7', "
    "absoluteFilePath=%8, canonicalFilePath=%9, exists=%10, isFile=%11, "
    "readable=%12, size=%13, open_ok=%14, qfile_error=%15, reason=%16")
    .arg(probe.label)
    .arg(probe.raw_path)
    .arg(probe.raw_path_length)
    .arg(probe.cleaned_path.isEmpty() ? QString("<empty>") : probe.cleaned_path)
    .arg(probe.cleaned_path.size())
    .arg(file_probe_detail::utf8_hex_dump(probe.cleaned_path))
    .arg(file_probe_detail::escaped_path_text(probe.cleaned_path))
    .arg(probe.absolute_path.isEmpty() ? QString("<empty>") : probe.absolute_path)
    .arg(probe.canonical_path.isEmpty() ? QString("<empty>") : probe.canonical_path)
    .arg(probe.exists ? "true" : "false")
    .arg(probe.is_file ? "true" : "false")
    .arg(probe.readable ? "true" : "false")
    .arg(probe.size)
    .arg(probe.open_ok ? "true" : "false")
    .arg(probe.error_string.isEmpty() ? QString("<empty>") : probe.error_string)
    .arg(probe.reason);
}

}  // namespace amr::visualization

#endif  // AMR_VISUALIZATION__FILE_PROBE_HPP_